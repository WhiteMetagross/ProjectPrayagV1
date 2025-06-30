# RoadMapper

A Python-based lane mapping system that extracts vehicle trajectories from traffic videos using YOLO object detection and converts them into lane polylines.

## Workings

RoadMapper transforms traffic videos into lane maps through a multi-stage process that analyzes vehicle movement patterns to infer road structure.

### 1. Vehicle Detection and Tracking

The system uses YOLO11 with oriented bounding boxes (OBB) to detect vehicles in each video frame. Unlike regular bounding boxes, OBB detection captures the actual orientation of vehicles, providing more accurate tracking data. The system specifically targets vehicle classes (cars and trucks) and assigns unique IDs to track individual vehicles across multiple frames using the BotSORT tracking algorithm.

### 2. Trajectory Extraction

For each detected vehicle, the system extracts the center point from its oriented bounding box. These center points are collected frame by frame to create continuous trajectories representing the path each vehicle traveled. The oriented bounding box approach ensures that even when vehicles are at angles or turning, the extracted center points accurately represent their movement.

### 3. Track Filtering and Validation

Raw trajectories are filtered based on quality metrics. Tracks must meet minimum duration requirements (default 3 seconds) to ensure they represent meaningful vehicle paths rather than brief detections. Additionally, tracks must contain sufficient data points to be considered valid. This filtering removes noise from brief false detections or vehicles that appear only momentarily in the frame.

### 4. Trajectory Smoothing

Vehicle trajectories naturally contain small fluctuations due to detection noise and actual vehicle movement variations. The system applies moving average smoothing using a configurable window size to create cleaner, more representative paths. This process maintains the overall trajectory shape while reducing minor irregularities.

### 5. Similar Track Merging

Multiple vehicles often follow nearly identical paths when traveling in the same lane. The system identifies these similar trajectories using the Hausdorff distance algorithm, which measures the maximum distance between corresponding points on two paths. Tracks with Hausdorff distances below a threshold are considered to represent the same lane and are merged together. This consolidation process combines multiple vehicle paths into single representative lane polylines.

### 6. Endpoint Connection

Real road lanes are continuous, but extracted trajectories may have gaps where vehicles enter or exit the camera's field of view. The endpoint snapping algorithm identifies trajectory endpoints that are close to each other (within a tolerance distance) and connects them, creating more complete lane representations. This step helps form continuous lane paths from fragmented vehicle trajectories.

### 7. Polyline Simplification

The final lane polylines may contain excessive detail that isn't necessary for lane representation. The system applies geometric simplification to reduce the number of points while preserving the essential shape of each lane. This creates cleaner, more manageable lane data without losing important geometric information.

## Core Algorithms Explained

* **Hausdorff Distance Calculation**: This algorithm measures how similar two vehicle paths are by finding the maximum distance any point on one path is from the nearest point on another path. It's calculated in both directions and takes the maximum value, ensuring robust similarity measurement even when paths have different lengths or sampling rates.
* **Moving Average Smoothing**: Each point in a trajectory is replaced with the average of surrounding points within a specified window. This reduces noise while preserving the overall path shape, making the resulting lanes more representative of actual road geometry.
* **Endpoint Snapping**: The algorithm examines the start and end points of all trajectories and identifies pairs that are within a specified distance tolerance. When close endpoints are found, they are merged to the same coordinate, effectively connecting separate trajectory segments into continuous lanes.
* **Track Merging Strategy**: When similar tracks are identified through Hausdorff distance comparison, they are combined by collecting all points from the similar trajectories and then applying smoothing to create a single representative path. This approach leverages multiple vehicle observations to create more accurate lane representations.

## Features

* Vehicle detection and tracking using YOLO11 with oriented bounding boxes (OBB)
* Automatic lane trajectory extraction from vehicle movements
* Track smoothing and merging of similar trajectories
* GeoJSON export for lane data
* Visualization video generation with overlaid lane polylines
* Configurable parameters for different traffic scenarios

## Requirements

* Python 3.7+
* YOLO11 model weights (yolo11m-obb.pt)
* BotSORT tracker configuration file

## Installation

1.  Clone this repository and then change into the directory:
    ```bash
    cd RoadMapper
    ```
2.  Install required dependencies:
    ```bash
    pip install -r requirements.txt
    ```
3.  Download YOLO11 OBB model weights and place them in your project directory.
4.  Ensure you have a BotSORT tracker YAML configuration file.

## Configuration

Edit `config.py` to set your file paths and parameters:

```python
inputVideo = "path/to/your/video.mp4"
outputDir = "path/to/output/directory"
model = "path/to/yolo11m-obb.pt"
trackerYAMLpath = "path/to/botsort.yaml"
````

### Key Parameters

  * `MIN_TRACK_DURATION_SECONDS`: Minimum duration for valid tracks (default: 3.0)
  * `HAUSDORFF_THRESHOLD`: Distance threshold for merging similar tracks (default: 20.0)
  * `ENDPOINT_SNAP_TOLERANCE`: Tolerance for connecting lane endpoints (default: 15.0)
  * `SIMPLIFY_TOLERANCE`: Tolerance for polyline simplification (default: 2.0)

## Usage

Run the main script:

```bash
python main.py
```

The system will:

1.  Process the input video and detect vehicles
2.  Extract vehicle trajectories using tracking
3.  Filter and smooth tracks based on duration and quality
4.  Merge similar trajectories into lane polylines
5.  Export results as GeoJSON and visualization video

## Output Files

  * `lanes.geojson`: Lane polylines in GeoJSON format
  * `processed_tracks.json`: Detailed track data and processing statistics
  * `lane_visualization.mp4`: Video with overlaid lane polylines

## File Structure

```
RoadMapper/
├── main.py                #Main orchestration script
├── config.py              #Configuration parameters
├── video_processing.py    #Video handling and YOLO detection
├── lane_processing.py     #Track processing and optimization
├── output_generation.py   #Export and visualization
├── requirements.txt       #Dependencies
├── botsort.yaml           #The botsort config yaml file
└── README.md              #This file
```

## Customization

  * Modify `YOLO_CLASSES` in config.py to detect different vehicle types
  * Adjust tracking parameters for different video conditions
  * Change visualization colors in `LANE_COLORS`
  * Tune processing thresholds for different traffic scenarios

## Troubleshooting

  * Ensure YOLO model weights are compatible with ultralytics version
  * Check video codec compatibility if visualization fails
  * Adjust minimum track duration for shorter/longer videos
  * Modify Hausdorff threshold if tracks aren't merging properly