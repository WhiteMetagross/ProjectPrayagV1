#This module processes video data to extract lane information from vehicle tracks.
#It generates GeoJSON files for lane representation and creates a visualization video showing the detected lanes.

import numpy as np
from scipy.spatial.distance import directed_hausdorff
from shapely.geometry import Point
import config

#Smoothing and merging tracks, snapping endpoints, and filtering valid tracks
def smooth_polyline(points, window_size=config.SMOOTHING_WINDOW_SIZE):
    if len(points) < window_size:
        return points
    
    coords = np.array(points)
    smoothed = []
    
    for i in range(len(coords)):
        start = max(0, i - window_size // 2)
        end = min(len(coords), i + window_size // 2 + 1)
        avg_point = np.mean(coords[start:end], axis=0)
        smoothed.append(avg_point.tolist())
    
    return smoothed

#Calculating Hausdorff distance between two polylines.
#This is used to determine if two tracks are similar enough to be merged.
def calculate_hausdorff_distance(line1, line2):
    points1 = np.array(line1)
    points2 = np.array(line2)
    
    if len(points1) < 2 or len(points2) < 2:
        return float('inf')
    
    d1 = directed_hausdorff(points1, points2)[0]
    d2 = directed_hausdorff(points2, points1)[0]
    
    return max(d1, d2)

#Merging similar tracks based on Hausdorff distance.
#Tracks that are within a certain threshold distance are merged into a single track.
def merge_similar_tracks(tracks, threshold=config.HAUSDORFF_THRESHOLD):
    if len(tracks) <= 1:
        return tracks
    
    merged = []
    used = set()
    
    for i, track1 in enumerate(tracks):
        if i in used:
            continue
            
        similar_tracks = [track1]
        used.add(i)
        
        for j, track2 in enumerate(tracks[i+1:], i+1):
            if j in used:
                continue
                
            if calculate_hausdorff_distance(track1, track2) < threshold:
                similar_tracks.append(track2)
                used.add(j)
        
        if len(similar_tracks) == 1:
            merged.append(track1)
        else:
            all_points = []
            for track in similar_tracks:
                all_points.extend(track)
            
            centroid_track = []
            for track in similar_tracks:
                for point in track:
                    centroid_track.append(point)
            
            merged.append(smooth_polyline(centroid_track))
    
    return merged

#Snapping endpoints of tracks to nearby tracks within a specified tolerance.
#This helps to connect tracks that are close to each other but not exactly aligned.
def snap_endpoints(tracks, tolerance=config.ENDPOINT_SNAP_TOLERANCE):
    if len(tracks) <= 1:
        return tracks
    
    snapped_tracks = []
    
    for track in tracks:
        if len(track) < 2:
            continue
            
        start_point = Point(track[0])
        end_point = Point(track[-1])
        
        snapped_track = track[:]
        
        for other_track in tracks:
            if other_track == track or len(other_track) < 2:
                continue
                
            other_start = Point(other_track[0])
            other_end = Point(other_track[-1])
            
            if start_point.distance(other_start) < tolerance:
                snapped_track[0] = other_track[0]
            elif start_point.distance(other_end) < tolerance:
                snapped_track[0] = other_track[-1]
                
            if end_point.distance(other_start) < tolerance:
                snapped_track[-1] = other_track[0]
            elif end_point.distance(other_end) < tolerance:
                snapped_track[-1] = other_track[-1]
        
        snapped_tracks.append(snapped_track)
    
    return snapped_tracks

#Filtering valid tracks based on minimum duration and number of points.
#Tracks that do not meet the criteria are removed.
def filter_valid_tracks(all_tracks, track_frame_counts, fps):
    min_frames = int(config.MIN_TRACK_DURATION_SECONDS * fps)
    valid_tracks = []
    
    for trackID, track_points in all_tracks.items():
        if track_frame_counts[trackID] >= min_frames and len(track_points) >= config.MIN_TRACK_POINTS:
            smoothed_track = smooth_polyline(track_points)
            valid_tracks.append(smoothed_track)
    
    return valid_tracks

#Main processing function that orchestrates the track processing steps.
#It filters valid tracks, merges similar tracks, and snaps endpoints.
def process_tracks(all_tracks, track_frame_counts, fps):
    valid_tracks = filter_valid_tracks(all_tracks, track_frame_counts, fps)
    print(f"Found {len(valid_tracks)} valid tracks (>= {config.MIN_TRACK_DURATION_SECONDS}s duration)")
    
    if len(valid_tracks) == 0:
        return []
    
    print("Merging similar tracks...")
    merged_tracks = merge_similar_tracks(valid_tracks)
    print(f"After merging: {len(merged_tracks)} tracks")
    
    print("Snapping endpoints...")
    snapped_tracks = snap_endpoints(merged_tracks)
    
    return snapped_tracks