#This module implements a lane aware vehicle tracking system using YOLO for detection and custom prediction logic for lane following vehicles.
#It tracks vehicles, predicts their future lane positions, and visualizes their paths on video frames.

from ultralytics import YOLO
import cv2
import os
import numpy as np
import colorsys
import math
from collections import deque
from lane_manager import LaneManager
from vehicle_predictor import VehiclePredictor
from config import PAST_HISTORY_SECONDS, FUTURE_PREDICTION_SECONDS, PROCESSING_FPS, TRAIL_LENGTH, PREDICTION_THICKNESS, ARROW_LENGTH

class LaneAwareVehicleTracker:
    def __init__(self, model_path, video_path, output_dir, tracker_config, lanes_geojson_path):

        #Initialize YOLO model for oriented bounding box detection.
        self.model = YOLO(model_path, task='obb')
        self.input_video = video_path
        self.output_dir = output_dir
        self.tracker_config = tracker_config
        
        #Extract video properties for output configurations.
        cap = cv2.VideoCapture(video_path)
        self.fps = cap.get(cv2.CAP_PROP_FPS)
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        
        #Initialize lane management and prediction components.
        self.lane_manager = LaneManager(lanes_geojson_path)
        self.predictor = VehiclePredictor(self.lane_manager)
        
        #Initialize tracking data structures.
        self.tracking_history = {}  #Stores position history for each vehicle.
        self.missed_frames = {}     #Counts consecutive frames without detection.
        self.lane_predictions = {}  #Stores active predictions for each vehicle.
        self.prediction_smoothing = {}  #Buffers recent predictions for temporal smoothing.
        self.MISS_THRESHOLD = 20    #Maximum frames before removing lost track.
        self.PREDICTION_INTERVAL = 8  #Frames between prediction updates.
        self.frame_count = 0 #Current frame index in the video.
        
        #Calculate history buffer size based on configured time duration.
        self.HISTORY_LENGTH = int(PAST_HISTORY_SECONDS * self.fps) + 5
    
    #Generate unique color for each vehicle track using golden ratio hashing.
    def get_color_for_track(self, track_id):
        track_id = int(track_id)
        hue = (track_id * 0.618033988749895) % 1.0
        r, g, b = colorsys.hsv_to_rgb(hue, 0.9, 0.9)
        return (int(r * 255), int(g * 255), int(b * 255))
    
    #Draw prediction paths with directional arrow, and a thickness based on probability.
    def draw_curved_path(self, img, path_points, color, probability, thickness=PREDICTION_THICKNESS):
        if len(path_points) < 2:
            return
        
        #Adjust visual properties based on prediction confidence.
        alpha = min(1.0, probability * 1.5)
        effective_thickness = max(2, int(thickness * alpha))
        
        #Filter points to stay within image boundaries
        points = []
        for point in path_points:
            x, y = int(point[0]), int(point[1])
            if 0 <= x < self.width and 0 <= y < self.height:
                points.append((x, y))
        
        if len(points) < 2:
            return
        
        #Draw the main path line.
        pts_array = np.array(points, dtype=np.int32)
        cv2.polylines(img, [pts_array], isClosed=False, color=color, thickness=effective_thickness)
        
        #Add directional arrow at path end.
        if len(points) >= 2:
            end_point = points[-1]
            second_last = points[-2]
            
            #Calculate arrow direction based on path end.
            angle = math.atan2(end_point[1] - second_last[1], end_point[0] - second_last[0])
            arrow_length = ARROW_LENGTH
            
            #Draw arrow head lines.
            arrow_p1 = (
                int(end_point[0] - arrow_length * math.cos(angle - math.pi / 5)),
                int(end_point[1] - arrow_length * math.sin(angle - math.pi / 5))
            )
            arrow_p2 = (
                int(end_point[0] - arrow_length * math.cos(angle + math.pi / 5)),
                int(end_point[1] - arrow_length * math.sin(angle + math.pi / 5))
            )
            
            cv2.line(img, end_point, arrow_p1, color, effective_thickness)
            cv2.line(img, end_point, arrow_p2, color, effective_thickness)
    
    #Apply temporal smoothing to reduce prediction clutter.
    def smooth_predictions(self, track_id, new_predictions):
        if track_id not in self.prediction_smoothing:
            self.prediction_smoothing[track_id] = []
        
        #Maintain buffer of recent predictions.
        self.prediction_smoothing[track_id].append(new_predictions)
        
        if len(self.prediction_smoothing[track_id]) > 2:
            self.prediction_smoothing[track_id].pop(0)
        
        #Return unmodified predictions if insufficient history.
        if len(self.prediction_smoothing[track_id]) == 1:
            return new_predictions
        
        #Average probabilities across recent predictions for same lanes.
        smoothed_predictions = []
        
        for pred_idx in range(min(len(pred) for pred in self.prediction_smoothing[track_id])):
            lane_id = new_predictions[pred_idx]['lane_id']
            
            all_paths = []
            all_probs = []
            
            #Collect matching lane predictions from recent history.
            for pred_set in self.prediction_smoothing[track_id]:
                if pred_idx < len(pred_set) and pred_set[pred_idx]['lane_id'] == lane_id:
                    all_paths.append(pred_set[pred_idx]['path_points'])
                    all_probs.append(pred_set[pred_idx]['probability'])
            
            #Create smoothed prediction with averaged probability
            if all_paths:
                avg_prob = sum(all_probs) / len(all_probs)
                
                current_pred = new_predictions[pred_idx].copy()
                current_pred['probability'] = avg_prob * 0.95  #Slight confidence decay
                smoothed_predictions.append(current_pred)
        
        return smoothed_predictions
    
    #Generate lane aware predictions for all vehicles with sufficient history.
    def predict_lane_trajectories(self):
        if len(self.tracking_history) < 1:
            return
        
        for track_id, history in self.tracking_history.items():
            #Require minimum history for stable predictions.
            if len(history) >= 10:
                try:
                    #Get raw predictions from the predictor.
                    predictions = self.predictor.predict_lane_possibilities(history, FUTURE_PREDICTION_SECONDS)
                    
                    if predictions:
                        #Apply temporal smoothing and store results.
                        smoothed_predictions = self.smooth_predictions(track_id, predictions)
                        
                        self.lane_predictions[track_id] = {
                            'predictions': smoothed_predictions,
                            'start_pos': list(history)[-1],
                            'timestamp': self.frame_count
                        }
                except Exception as e:
                    continue
    
    #Render all active predictions with color coded probability ranking
    def draw_lane_predictions(self, img):
        current_time = self.frame_count
        
        #Define colors for top 3 predictions (green, orange, cyan).
        colors = [
            (0, 255, 0),
            (255, 165, 0),
            (0, 255, 255)
        ]
        
        #Process each vehicle's predictions.
        for track_id, pred_data in list(self.lane_predictions.items()):
            #Remove stale predictions.
            if current_time - pred_data['timestamp'] > 15:
                del self.lane_predictions[track_id]
                if track_id in self.prediction_smoothing:
                    del self.prediction_smoothing[track_id]
                continue
            
            predictions = pred_data['predictions']
            
            #Draw up to 3 predictions per vehicle.
            for i, pred in enumerate(predictions):
                if i >= len(colors):
                    break
                
                color = colors[i]
                probability = pred['probability']
                path_points = pred['path_points']
                
                #Only draw predictions above minimum confidence threshold.
                if len(path_points) > 1 and probability > 0.3:
                    self.draw_curved_path(img, path_points, color, probability)
    
    #Render lane geometry as reference lines.
    def draw_lanes(self, img):
        lane_color = (100, 100, 100)  #Gray color for lanes.
        
        for lane in self.lane_manager.lanes:
            coords = lane['coords']
            if len(coords) > 1:
                #Filter coordinates to image boundaries.
                pts = []
                for coord in coords:
                    x, y = int(coord[0]), int(coord[1])
                    if 0 <= x < self.width and 0 <= y < self.height:
                        pts.append((x, y))
                
                #Draw lane as polyline.
                if len(pts) > 1:
                    pts_np = np.array(pts, dtype=np.int32)
                    cv2.polylines(img, [pts_np], isClosed=False, color=lane_color, thickness=1)
    
    #Main video processing loop with tracking and prediction.
    def process_video(self):
        os.makedirs(self.output_dir, exist_ok=True)
        output_path = os.path.join(self.output_dir, "CleanLanePredictor.mp4")
        
        #Setup video writer with same properties as input.
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, self.fps, (self.width, self.height))
        
        #Print configuration summary.
        print(f"Starting clean lane-aware vehicle tracking...")
        print(f"Using {PAST_HISTORY_SECONDS} seconds of history to predict {FUTURE_PREDICTION_SECONDS} seconds into future")
        print(f"Loaded {len(self.lane_manager.lanes)} lane polylines for guidance")
        
        try:
            #Start YOLO11-OBB with BoTSort tracking on video stream.
            results = self.model.track(
                self.input_video,
                show=False,
                show_labels=False,
                show_conf=False,
                line_width=2,
                classes=[9, 10],      #Filter for vehicle classes only.
                tracker=self.tracker_config,
                save=False,
                stream=True,
                conf=0.25,            #Detection confidence threshold.
                verbose=False
            )
            
            #Process each frame from the video stream.
            for result in results:
                self.frame_count += 1
                
                #Get frame with YOLO detections drawn.
                plotted_img = result.plot(labels=False, conf=False, line_width=2)
                
                #Draw lane reference geometry.
                self.draw_lanes(plotted_img)
                
                #Extract vehicle positions and update tracking.
                current_centers = []
                current_track_ids = set()
                
                #Process detected vehicles if any exist.
                if hasattr(result, 'obb') and result.obb is not None:
                    for idx, box in enumerate(result.obb):
                        #Extract vehicle center position.
                        cx = int(box.xywhr[0][0].item())
                        cy = int(box.xywhr[0][1].item())
                        current_centers.append((cx, cy))
                        
                        #Get or assign track ID.
                        track_id = getattr(box, 'id', None)
                        if track_id is None:
                            track_id = idx
                        else:
                            track_id = int(track_id.item())
                        current_track_ids.add(track_id)
                        
                        #Initialize or update position history
                        if track_id not in self.tracking_history:
                            self.tracking_history[track_id] = deque(maxlen=self.HISTORY_LENGTH)
                        
                        self.tracking_history[track_id].append((cx, cy))
                        self.missed_frames[track_id] = 0
                
                #Handle missing detections and track cleanup.
                for track_id in list(self.tracking_history.keys()):
                    if track_id not in current_track_ids:
                        #Increment miss counter for absent tracks.
                        self.missed_frames[track_id] = self.missed_frames.get(track_id, 0) + 1
                        #Remove tracks that have been missing too long.
                        if self.missed_frames[track_id] > self.MISS_THRESHOLD:
                            del self.tracking_history[track_id]
                            del self.missed_frames[track_id]
                            if track_id in self.lane_predictions:
                                del self.lane_predictions[track_id]
                            if track_id in self.prediction_smoothing:
                                del self.prediction_smoothing[track_id]
                
                #Generate predictions at regular intervals.
                if self.frame_count % self.PREDICTION_INTERVAL == 0:
                    self.predict_lane_trajectories()
                
                #Draw historical trails for each tracked vehicle.
                for track_id, pts in self.tracking_history.items():
                    color = self.get_color_for_track(track_id)
                    if len(pts) >= 2:
                        #Draw recent trail segment.
                        recent_pts = list(pts)[-min(TRAIL_LENGTH, len(pts)):]
                        pts_np = np.array(recent_pts, dtype=np.int32)
                        cv2.polylines(plotted_img, [pts_np], isClosed=False, color=color, thickness=2)
                
                #Render all active predictions.
                self.draw_lane_predictions(plotted_img)
                
                #Draw current vehicle center points.
                for (cx, cy) in current_centers:
                    cv2.circle(plotted_img, (cx, cy), radius=4, color=(0, 0, 255), thickness=-1)
                
                #Add status information overlay
                cv2.putText(plotted_img, f"Frame: {self.frame_count}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(plotted_img, f"Tracked: {len(self.tracking_history)}", (10, 55), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(plotted_img, f"Predictions: {len(self.lane_predictions)}", (10, 80), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                #Write processed frame to output video.
                out.write(plotted_img)
                
                #Print progress updates.
                if self.frame_count % 100 == 0:
                    print(f"Processed {self.frame_count} frames...")
        
        except Exception as e:
            print(f"Error during processing: {e}")
        finally:
            #Cleanup and summary.
            out.release()
            
            print(f"Clean lane tracking complete! Output saved to: {output_path}")
            print(f"Total frames processed: {self.frame_count}")
            print(f"Total unique vehicles tracked: {len(self.tracking_history)}")