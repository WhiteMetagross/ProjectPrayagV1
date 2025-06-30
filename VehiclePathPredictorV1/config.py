#The configuration constants and file paths for vehicle path prediction system.

PAST_HISTORY_SECONDS = 4.0
FUTURE_PREDICTION_SECONDS = 2.0
PROCESSING_FPS = 50

#Paths to required files and directories.
MODEL_PATH = r"C:\Users\Xeron\Videos\PrayagIntersection\yolo11m-obb.pt"
VIDEO_PATH = r"C:\Users\Xeron\Videos\PrayagIntersection\PrayagIntersection1.mp4"
PROJECT_PATH = r"C:\Users\Xeron\Videos\PrayagIntersection"
TRACKER_CONFIG = r"C:\Users\Xeron\OneDrive\Documents\Programs\PrayagProject\botsort.yaml"
LANES_GEOJSON = r"C:\Users\Xeron\Videos\PrayagIntersection\lane_mapping\lanes.geojson"
OUTPUT_DIR_NAME = "clean_lane_prediction"

TRAIL_LENGTH = 75
PREDICTION_THICKNESS = 2
ARROW_LENGTH = 12