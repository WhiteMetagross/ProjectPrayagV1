#This is the main entry point for the RoadMapper application.
#It initializes the video processing, lane extraction, and output generation modules.
#It processes the input video to extract lane information and generates the required outputs in GeoJSON and JSON formats.

import os
import config
from video_processing import get_video_info, process_video_tracks
from lane_processing import process_tracks
from output_generation import export_geojson, export_tracks_json, create_visualization_video

def main():
    os.makedirs(config.outputDir, exist_ok=True)
    
    video_info = get_video_info(config.inputVideo)
    print(f"Video info: {video_info['width']}x{video_info['height']}, {video_info['fps']} fps, {video_info['total_frames']} frames")
    
    all_tracks, track_frame_counts, detection_count = process_video_tracks()
    
    processed_tracks = process_tracks(all_tracks, track_frame_counts, video_info["fps"])
    
    if len(processed_tracks) == 0:
        print("No valid tracks found! Check your video file and parameters.")
        return
    
    print("Creating GeoJSON...")
    geojson_path = export_geojson(processed_tracks, video_info)
    
    tracks_json_path = export_tracks_json(
        processed_tracks, 
        video_info, 
        detection_count, 
        len(all_tracks), 
        len(processed_tracks), 
        len(processed_tracks)
    )
    
    visualization_path = create_visualization_video(processed_tracks, video_info)
    
    print(f"Lane mapping complete!")
    print(f"GeoJSON saved to: {geojson_path}")
    print(f"Processed tracks saved to: {tracks_json_path}")
    print(f"Visualization video saved to: {visualization_path}")
    print(f"Final results:")
    print(f"  - Total detections: {detection_count}")
    print(f"  - Total tracks: {len(all_tracks)}")
    print(f"  - Final lane polylines: {len(processed_tracks)}")

if __name__ == "__main__":
    main()