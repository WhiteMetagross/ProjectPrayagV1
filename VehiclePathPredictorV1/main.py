#This program initializes the vehicle path predictor and processes the video to predict vehicle paths.
#It's the entry point for the vehicle path prediction system, checking for required files and running the tracker.
#It uses the LaneAwareVehicleTracker class to handle video processing and path prediction.

import os
import traceback
from tracker import LaneAwareVehicleTracker
from config import (PAST_HISTORY_SECONDS, FUTURE_PREDICTION_SECONDS, 
                   MODEL_PATH, VIDEO_PATH, PROJECT_PATH, TRACKER_CONFIG, 
                   LANES_GEOJSON, OUTPUT_DIR_NAME)

if __name__ == "__main__":
    print("Checking required files...")
    files_to_check = [MODEL_PATH, VIDEO_PATH, TRACKER_CONFIG, LANES_GEOJSON]
    for file_path in files_to_check:
        if os.path.exists(file_path):
            print(f"✅ Found: {os.path.basename(file_path)}")
        else:
            print(f"❌ Missing: {file_path}")
    
    output_dir = os.path.join(PROJECT_PATH, OUTPUT_DIR_NAME)
    
    try:
        tracker = LaneAwareVehicleTracker(
            model_path=MODEL_PATH,
            video_path=VIDEO_PATH,
            output_dir=output_dir,
            tracker_config=TRACKER_CONFIG,
            lanes_geojson_path=LANES_GEOJSON
        )
        
        tracker.process_video()
        print("Clean lane processing completed successfully.")
        print(f"Prediction configuration: {PAST_HISTORY_SECONDS} seconds past → {FUTURE_PREDICTION_SECONDS} seconds future")
        print("🟢 Green paths show most likely destinations")
        print("🟠 Orange paths show secondary destinations")
        print("🔵 Cyan paths show third destinations")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        traceback.print_exc()