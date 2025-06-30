#This module processes video data to extract lane information from vehicle tracks.
#It generates GeoJSON files for lane representation and creates a visualization video showing the detected lanes.

from ultralytics import YOLO
import cv2
import numpy as np
from collections import defaultdict
import config

def get_video_info(video_path):
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    
    return {
        "fps": fps,
        "width": width,
        "height": height,
        "total_frames": total_frames
    }

def extract_obb_polyline(obb_data):
    cx, cy, w, h, angle = obb_data.xywhr[0].cpu().numpy()
    
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    
    half_w = w / 2
    half_h = h / 2
    
    corners = np.array([
        [-half_w, -half_h],
        [half_w, -half_h],
        [half_w, half_h],
        [-half_w, half_h]
    ])
    
    rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
    rotated_corners = corners @ rotation_matrix.T
    
    final_corners = rotated_corners + np.array([cx, cy])
    
    return final_corners.tolist()

def process_video_tracks():
    model = YOLO(config.model)
    
    all_tracks = defaultdict(list)
    track_frame_counts = defaultdict(int)
    
    results = model.track(
        config.inputVideo,
        show=False,
        show_labels=False,
        show_conf=False,
        line_width=config.LINE_WIDTH,
        classes=config.YOLO_CLASSES,
        tracker=config.trackerYAMLpath,
        save=False,
        stream=True,
        conf=config.YOLO_CONF,
        iou=config.YOLO_IOU
    )
    
    print("Processing video and extracting OBB polylines...")
    frame_count = 0
    detection_count = 0
    
    for result in results:
        frame_count += 1
        
        if hasattr(result, 'obb') and result.obb is not None:
            for box in result.obb:
                detection_count += 1
                
                trackID = getattr(box, 'id', None)
                if trackID is not None:
                    trackID = int(trackID.item())
                    
                    polyline = extract_obb_polyline(box)
                    center_point = [
                        sum(p[0] for p in polyline) / len(polyline),
                        sum(p[1] for p in polyline) / len(polyline)
                    ]
                    
                    all_tracks[trackID].append(center_point)
                    track_frame_counts[trackID] += 1
        
        if frame_count % 100 == 0:
            print(f"Processed {frame_count} frames, {detection_count} detections, {len(all_tracks)} tracks")
    
    print(f"Total processing complete: {frame_count} frames, {detection_count} detections")
    
    return all_tracks, track_frame_counts, detection_count