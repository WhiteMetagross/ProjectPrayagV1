#Configuration parameters and file paths for RoadMapper.

import random

#The file paths for the input video, output directory, model, and tracker configuration.
inputVideo = r"C:\Users\Xeron\Videos\PrayagIntersection\PrayagIntersection1.mp4"
outputDir = r"C:\Users\Xeron\Videos\PrayagIntersection\lane_mapping"
model = r"C:\Users\Xeron\Videos\PrayagIntersection\yolo11m-obb.pt"
trackerYAMLpath = r"C:\Users\Xeron\OneDrive\Documents\Programs\PrayagProject\botsort.yaml"

MIN_TRACK_DURATION_SECONDS = 3.0
HAUSDORFF_THRESHOLD = 20.0
ENDPOINT_SNAP_TOLERANCE = 15.0
SIMPLIFY_TOLERANCE = 2.0

YOLO_CLASSES = [9, 10]
YOLO_CONF = 0.05
YOLO_IOU = 0.5
LINE_WIDTH = 2

SMOOTHING_WINDOW_SIZE = 5
MIN_TRACK_POINTS = 5

#The function to generate random colors for lane visualization.
def generate_random_colors(num_colors):
    colors = []
    for _ in range(num_colors):
        color = (
            random.randint(0, 255),
            random.randint(0, 255),
            random.randint(0, 255)
        )
        colors.append(color)
    return colors

LANE_COLORS = generate_random_colors(50)