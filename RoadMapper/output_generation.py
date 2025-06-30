#This module processes video data to extract lane information from vehicle tracks. 
#It generates GeoJSON files for lane representation and creates a visualization video showing the detected lanes.
#It also exports processed tracks to a JSON file containing video metadata and processing statistics.

import cv2
import os
import json
import numpy as np
import geojson
from shapely.geometry import LineString
import config

def create_geojson_from_tracks(tracks, video_info):
    features = []
    
    for i, track in enumerate(tracks):
        if len(track) < 2:
            continue
            
        try:
            line = LineString(track)
            simplified_line = line.simplify(config.SIMPLIFY_TOLERANCE, preserve_topology=True)
            
            feature = geojson.Feature(
                geometry=geojson.LineString(list(simplified_line.coords)),
                properties={
                    "lane_id": i,
                    "length": simplified_line.length,
                    "point_count": len(track)
                }
            )
            features.append(feature)
        except Exception as e:
            print(f"Error creating feature for track {i}: {e}")
            continue
    
    feature_collection = geojson.FeatureCollection(
        features,
        properties={
            "video_width": video_info["width"],
            "video_height": video_info["height"],
            "fps": video_info["fps"],
            "total_frames": video_info["total_frames"],
            "total_lanes": len(features)
        }
    )
    
    return feature_collection

def export_geojson(tracks, video_info):
    geojson_data = create_geojson_from_tracks(tracks, video_info)
    geojson_path = os.path.join(config.outputDir, "lanes.geojson")
    
    with open(geojson_path, 'w') as f:
        json.dump(geojson_data, f, indent=2)
    
    return geojson_path

def export_tracks_json(tracks, video_info, detection_count, all_tracks_count, valid_tracks_count, merged_tracks_count):
    tracks_json_path = os.path.join(config.outputDir, "processed_tracks.json")
    
    with open(tracks_json_path, 'w') as f:
        json.dump({
            "video_info": video_info,
            "tracks": tracks,
            "processing_stats": {
                "total_detections": detection_count,
                "total_tracks": all_tracks_count,
                "valid_tracks": valid_tracks_count,
                "merged_tracks": merged_tracks_count,
                "final_lanes": len(tracks)
            }
        }, f, indent=2)
    
    return tracks_json_path

def create_visualization_video(tracks, video_info):
    print("Creating visualization video...")
    visualization_path = os.path.join(config.outputDir, "lane_visualization.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(visualization_path, fourcc, video_info["fps"], (video_info["width"], video_info["height"]))
    
    cap = cv2.VideoCapture(config.inputVideo)
    frame_idx = 0
    
    lane_colors = config.generate_random_colors(len(tracks))
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        overlay = frame.copy()
        
        for i, track in enumerate(tracks):
            if len(track) < 2:
                continue
                
            color = lane_colors[i % len(lane_colors)]
            
            pts = np.array(track, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(overlay, [pts], False, color, 3)
        
        cv2.putText(overlay, f"Lanes: {len(tracks)}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(overlay, f"Frame: {frame_idx}/{video_info['total_frames']}", (10, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        out.write(overlay)
        frame_idx += 1
        
        if frame_idx % 100 == 0:
            print(f"Rendered {frame_idx} frames...")
    
    cap.release()
    out.release()
    
    return visualization_path