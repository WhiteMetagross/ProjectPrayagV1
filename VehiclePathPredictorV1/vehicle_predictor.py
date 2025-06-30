#This module implements a vehicle path prediction system that uses lane information to predict future vehicle 
#positions based on historical movement data.
#It integrates with a lane manager to access lane geometry and uses a custom prediction algorithm to generate 
#ranked future paths for vehicles.
#The predictions are based on the vehicle's recent movement history, velocity, and alignment with nearby lanes.


import math
from config import FUTURE_PREDICTION_SECONDS, PROCESSING_FPS

class VehiclePredictor:
    #Store reference to lane manager for geometric operations.
    def __init__(self, lane_manager):
        self.lane_manager = lane_manager

    #The main prediction function which returns ranked list of possible future paths.   
    def predict_lane_possibilities(self, history, time_horizon=FUTURE_PREDICTION_SECONDS):
        if len(history) < 8:
            return []
            
        positions = list(history)
        current_pos = positions[-1]
        
        #Calculate smoothed velocity from recent position changes.
        smoothed_velocity = self._calculate_smoothed_velocity(positions)
        if smoothed_velocity is None or self._get_speed(smoothed_velocity) < 0.5:
            return []
        
        #Determine overall movement direction from position history.
        movement_direction = self._get_movement_direction(positions)
        if movement_direction is None:
            return []
        
        #Find candidate lanes within reasonable distance.
        nearby_lanes = self.lane_manager.get_lanes_within_distance(current_pos, 80)
        if not nearby_lanes:
            return []
        
        lane_predictions = []
        
        #Evaluate each nearby lane as a potential future path.
        for lane_info in nearby_lanes[:4]:  #Limit to 4 closest lanes for performance.
            lane = lane_info['lane']
            distance_to_lane = lane_info['distance']
            progress = lane_info['progress']
            
            #Skip lanes that are too far away.
            if distance_to_lane > 50:
                continue
            
            #Get lane direction at vehicle's closest point.
            lane_direction = self.lane_manager.get_lane_direction_at_progress(lane, progress)
            if lane_direction is None:
                continue
            
            #Calculate alignment between vehicle movement and lane direction.
            alignment = self._calculate_alignment(movement_direction, lane_direction)
            if alignment < 0.3:  #Skip poorly aligned lanes.
                continue
            
            #Calculate overall probability for this lane
            probability = self._calculate_lane_probability(
                positions, lane, distance_to_lane, alignment, movement_direction
            )
            
            #Generate prediction path if probability is sufficient.
            if probability > 0.2:
                #Estimate how far vehicle will travel in prediction time.
                prediction_distance = min(
                    self._estimate_travel_distance(smoothed_velocity, time_horizon),
                    120  #Cap maximum prediction distance.
                )
                #Get smooth path along the lane.
                path_points = self.lane_manager.get_path_along_lane(lane, progress, prediction_distance)
                
                #Store complete prediction data.
                if len(path_points) > 1:
                    lane_predictions.append({
                        'lane_id': lane['id'],
                        'probability': probability,
                        'path_points': path_points,
                        'lane_direction': lane_direction,
                        'alignment': alignment,
                        'distance_to_lane': distance_to_lane,
                        'travel_distance': prediction_distance
                    })
        
        #Return top three predictions sorted by probability.s
        lane_predictions.sort(key=lambda x: x['probability'], reverse=True)
        return lane_predictions[:3]

    #Compute average velocity from recent position changes to reduce noise.
    def _calculate_smoothed_velocity(self, positions):
        if len(positions) < 4:
            return None
        
        velocities = []
        dt = 1.0 / PROCESSING_FPS  #Time between frames.
        
        #Calculate velocity for each recent frame pair.
        for i in range(len(positions) - 3, len(positions)):
            if i > 0:
                vel_x = (positions[i][0] - positions[i-1][0]) / dt
                vel_y = (positions[i][1] - positions[i-1][1]) / dt
                velocities.append([vel_x, vel_y])
        
        if not velocities:
            return None
        
        #Return averaged velocity components.
        avg_vel_x = sum(v[0] for v in velocities) / len(velocities)
        avg_vel_y = sum(v[1] for v in velocities) / len(velocities)
        
        return [avg_vel_x, avg_vel_y]

    #Calculate normalized direction vector from recent position history.  
    def _get_movement_direction(self, positions):
        if len(positions) < 5:
            return None
        
        #Use positions from 5 frames ago to current for stable direction.
        start_pos = positions[-5]
        end_pos = positions[-1]
        
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]
        
        #Require minimum movement to avoid noise.
        length = math.sqrt(dx*dx + dy*dy)
        if length < 2:
            return None
        
        #Return normalized direction vector.
        return [dx/length, dy/length]
    
    #Calculate speed magnitude from velocity vector.
    def _get_speed(self, velocity):
        return math.sqrt(velocity[0]**2 + velocity[1]**2)

    #Compute alignment score between vehicle movement and lane direction
    def _calculate_alignment(self, movement_direction, lane_direction):
        dot_product = abs(movement_direction[0] * lane_direction[0] + movement_direction[1] * lane_direction[1])
        return dot_product

    #Compute overall probability score for vehicle following this lane.
    def _calculate_lane_probability(self, positions, lane, distance_to_lane, alignment, movement_direction):
        base_probability = alignment
        
        #Reduce probability based on distance from lane.
        distance_factor = max(0, 1 - (distance_to_lane / 50))
        base_probability *= distance_factor
        
        #Boost probability for vehicles with consistent movement patterns.
        consistency = self._calculate_direction_consistency(positions)
        base_probability *= (0.5 + 0.5 * consistency)
        
        #Significant boost for vehicles very close to lane centerline.
        if distance_to_lane < 15:
            base_probability *= 1.5
        
        #Ensure probability stays within valid range.
        return min(1.0, base_probability)

    #Measure how consistently vehicle has been moving in same direction.    
    def _calculate_direction_consistency(self, positions):
        if len(positions) < 6:
            return 0.5
        
        directions = []
        #Calculate direction vectors for recent frame pairs.
        for i in range(len(positions) - 3, len(positions)):
            if i > 0:
                dx = positions[i][0] - positions[i-1][0]
                dy = positions[i][1] - positions[i-1][1]
                length = math.sqrt(dx*dx + dy*dy)
                if length > 0:
                    directions.append([dx/length, dy/length])
        
        if len(directions) < 2:
            return 0.5
        
        #Calculate average alignment between consecutive directions.
        consistency_sum = 0
        count = 0
        
        for i in range(1, len(directions)):
            dot = directions[i][0] * directions[i-1][0] + directions[i][1] * directions[i-1][1]
            consistency_sum += max(0, dot)  #Only positive alignments count
            count += 1
        
        return consistency_sum / count if count > 0 else 0.5
    
    #Estimate how far vehicle will travel in the prediction time horizon.
    def _estimate_travel_distance(self, velocity, time_horizon):
        speed = self._get_speed(velocity)
        #Apply scaling factor to account for real-world vs pixel coordinates.
        return speed * time_horizon * PROCESSING_FPS * 0.6