# This module provides a LaneManager class to handle lane data and operations.
# It loads lane data from a GeoJSON file, finds the closest lane to a point,
# and provides methods to get lane direction, forward position, and paths along lanes.

import os
import json
import math
from shapely.geometry import LineString, Point
from shapely.ops import nearest_points

# LaneManager class to manage lane data and operations
# This class loads lane data from a GeoJSON file, finds the closest lane to a point
# and provides methods to get lane direction, forward position, and paths along lanes.
class LaneManager:
    def __init__(self, geojson_path):
        self.lanes = []
        self.load_lanes(geojson_path)
    
    #Load lanes from a GeoJSON file.
    #The GeoJSON file should contain features with 'lane_id' and 'coordinates'.
    def load_lanes(self, geojson_path):
        if not os.path.exists(geojson_path):
            print(f"Warning: Lane file not found at {geojson_path}")
            return
            
        with open(geojson_path, 'r') as f:
            data = json.load(f)
            
        for feature in data['features']:
            coords = feature['geometry']['coordinates']
            lane_id = feature['properties']['lane_id']
            
            self.lanes.append({
                'id': lane_id,
                'coords': coords,
                'linestring': LineString(coords)
            })
        
        print(f"Loaded {len(self.lanes)} lane polylines")
    
    #Find the closest lane to a given point.
    #Returns the closest lane, the closest point on the lane, and the progress along the lane.
    #If no lanes are available, returns None for all values.
    def find_closest_lane(self, point):
        if not self.lanes:
            return None, None, None
            
        min_distance = float('inf')
        closest_lane = None
        closest_point = None
        progress = None
        
        point_geom = Point(point)
        
        for lane in self.lanes:
            distance = point_geom.distance(lane['linestring'])
            
            if distance < min_distance:
                min_distance = distance
                closest_lane = lane
                closest_point_geom = nearest_points(point_geom, lane['linestring'])[1]
                closest_point = [closest_point_geom.x, closest_point_geom.y]
                progress = lane['linestring'].project(point_geom)
        
        return closest_lane, closest_point, progress
    
    #Get lanes within a specified distance from a point.
    #Returns a list of nearby lanes with their distance, closest point, and progress.
    def get_lanes_within_distance(self, point, max_distance=60):
        point_geom = Point(point)
        nearby_lanes = []
        
        for lane in self.lanes:
            distance = point_geom.distance(lane['linestring'])
            if distance <= max_distance:
                closest_point_geom = nearest_points(point_geom, lane['linestring'])[1]
                progress = lane['linestring'].project(point_geom)
                nearby_lanes.append({
                    'lane': lane,
                    'distance': distance,
                    'closest_point': [closest_point_geom.x, closest_point_geom.y],
                    'progress': progress
                })
        
        return sorted(nearby_lanes, key=lambda x: x['distance'])
    
    #Get the direction of a lane at a given progress.
    #Returns a normalized direction vector [dx, dy] or None if the lane is invalid.
    def get_lane_direction_at_progress(self, lane, progress):
        if not lane or progress is None:
            return None
            
        coords = lane['coords']
        if len(coords) < 2:
            return None
            
        total_length = lane['linestring'].length
        if total_length == 0:
            return None
            
        segment_lengths = []
        cumulative_length = 0
        
        for i in range(len(coords) - 1):
            seg_length = math.sqrt((coords[i+1][0] - coords[i][0])**2 + 
                                 (coords[i+1][1] - coords[i][1])**2)
            segment_lengths.append(seg_length)
            cumulative_length += seg_length
        
        if cumulative_length == 0:
            return None
            
        target_length = (progress / total_length) * cumulative_length
        current_length = 0
        
        for i, seg_length in enumerate(segment_lengths):
            if current_length + seg_length >= target_length:
                dx = coords[i+1][0] - coords[i][0]
                dy = coords[i+1][1] - coords[i][1]
                
                if dx == 0 and dy == 0:
                    continue
                    
                length = math.sqrt(dx*dx + dy*dy)
                return [dx/length, dy/length]
            
            current_length += seg_length
        
        if len(coords) >= 2:
            dx = coords[-1][0] - coords[-2][0]
            dy = coords[-1][1] - coords[-2][1]
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                return [dx/length, dy/length]
        
        return None
    
    #Get the forward position along a lane at a given progress and distance.
    #Returns the new position [x, y] or None if the operation fails.
    def get_forward_position(self, lane, progress, distance):
        try:
            new_progress = progress + distance
            point = lane['linestring'].interpolate(new_progress)
            return [point.x, point.y]
        except:
            return None
    
    #Get a path along a lane from a starting progress to a specified distance.
    #Returns a list of points along the path or an empty list if the operation fails.
    def get_path_along_lane(self, lane, start_progress, distance, num_points=8):
        try:
            path_points = []
            end_progress = start_progress + distance
            
            if lane['linestring'].length <= start_progress:
                return []
            
            max_progress = min(end_progress, lane['linestring'].length)
            
            if max_progress <= start_progress:
                return []
                
            progress_step = (max_progress - start_progress) / max(1, num_points - 1)
            
            for i in range(num_points):
                current_progress = min(start_progress + (i * progress_step), lane['linestring'].length)
                point = lane['linestring'].interpolate(current_progress)
                path_points.append([point.x, point.y])
                
                if current_progress >= max_progress:
                    break
            
            return path_points
        except:
            return []