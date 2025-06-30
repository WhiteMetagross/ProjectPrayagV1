# Vehicle Path Predictor

A lane-aware vehicle tracking and path prediction system that combines YOLO11-OBB object detection, multi-object tracking, and geometric lane analysis to forecast future vehicle trajectories with high accuracy.

## Overview

This system processes traffic videos to predict where vehicles will go next by analyzing their movement patterns and constraining predictions to realistic lane-following behavior. It maintains a continuous history of vehicle positions and uses this data to generate probabilistic forecasts of future paths. By incorporating actual lane geometry, predictions are automatically constrained to follow realistic paths rather than simple straight-line extrapolations.

## Key Features

-   **Real-time Vehicle Detection**: Uses a YOLO model with oriented bounding boxes to accurately detect trucks and buses.
-   **Multi-Path Trajectory Prediction**: Generates up to 3 possible future paths per vehicle, each with a confidence score.
-   **Lane-Constrained Forecasting**: Utilizes pre-mapped lane geometry to ensure all predictions are realistic and follow the road's structure.
-   **Temporal Smoothing**: Reduces visual jitter in predictions through multi-frame averaging for a more stable output.
-   **Adaptive Tracking**: Automatically handles vehicle occlusion (when a vehicle is temporarily hidden) and re-identifies vehicles correctly.
-   **Historical Path Tracking**: Maintains a history of vehicle movements to improve the accuracy and context of predictions.
-   **Smooth Trajectory Visualization**: Displays predicted paths with clear directional arrows and styling based on the calculated probability.

## How the Prediction Algorithm Works

### 1. Vehicle Detection and Tracking
The system begins by detecting vehicles in each video frame using a YOLO model trained specifically for oriented bounding box detection. This provides more accurate vehicle positioning compared to standard rectangular boxes. Each detected vehicle is assigned an unique tracking ID that persists across frames, allowing the system to build a continuous movement history. The tracking component maintains a rolling buffer of vehicle positions over the past 4 seconds (configurable). When vehicles temporarily disappear due to occlusion or detection failures, the system continues tracking for up to 20 frames before declaring the track lost.

### 2. Movement Analysis
For each tracked vehicle, the system analyzes recent movement patterns to extract key motion parameters:

* **Velocity Calculation**: The algorithm computes smoothed velocity vectors by averaging the last several position changes. This reduces noise from detection jitter while capturing the vehicle's true motion direction and speed.
* **Direction Consistency**: The system evaluates how consistently the vehicle has been moving in its current direction. Vehicles with erratic or inconsistent movement receive lower prediction confidence scores.
* **Speed Filtering**: Very slow-moving or stationary vehicles are excluded from prediction since their future paths are highly uncertain.

### 3. Lane Matching and Alignment
The core idea lies in how the system uses pre-mapped lane geometry to constrain predictions:

* **Proximity Analysis**: For each vehicle, the system identifies all lane segments within 80 pixels of the current position. This creates a candidate set of possible lanes the vehicle might follow.
* **Direction Alignment**: The algorithm computes the alignment between the vehicle's movement direction and each candidate lane's direction at the closest point. This is done using vector dot products to measure angular similarity.
* **Progress Tracking**: The system calculates how far along each lane the vehicle has progressed, enabling it to predict continuation along the same lane or transitions to connected lanes.

### 4. Probability Scoring
Each potential lane receives a probability score based on multiple factors:

* **Geometric Alignment**: Lanes that closely match the vehicle's current direction receive higher scores. The system uses the absolute value of the dot product between normalized direction vectors.
* **Distance Weighting**: Closer lanes receive higher probability scores, with the influence decreasing exponentially with distance.
* **Movement Consistency**: Vehicles with consistent directional movement over time receive bonus probability scores, while erratic movers are penalized.
* **Lane Preference**: Vehicles already very close to a lane centerline (within 15 pixels) receive significant probability boosts for continuing on that lane.

### 5. Path Generation
Once promising lanes are identified and scored, the system generates smooth trajectory paths:

* **Travel Distance Estimation**: The algorithm estimates how far the vehicle will travel in the prediction time horizon (typically 2 seconds) based on its current velocity and historical speed patterns.
* **Lane Following**: For each high-probability lane, the system traces a path along the lane geometry starting from the vehicle's current position and extending for the estimated travel distance.
* **Curve Interpolation**: The generated paths follow the natural curves of the lane geometry, creating realistic trajectory predictions that account for road curvature and lane changes.

### 6. Temporal Smoothing
To reduce visual jitter and improve prediction stability, the system maintains a short history of recent predictions and averages them to create smoother, more stable forecasts. Prediction confidence gradually decreases over time, preventing old predictions from persisting too long. New predictions are generated every 8 frames (configurable), balancing computational efficiency with prediction freshness.

## Algorithm Strengths

* **Realistic Constraints**: By incorporating actual lane geometry, predictions are automatically constrained to follow realistic paths rather than simple straight-line extrapolations.
* **Multi-Hypothesis Tracking**: The system can simultaneously consider multiple possible futures for each vehicle, capturing the uncertainty inherent in predicting human driving behavior.
* **Adaptive Confidence**: Prediction confidence automatically adjusts based on movement consistency, lane alignment, and other factors, providing honest uncertainty estimates.
* **Computational Efficiency**: The algorithm is designed to run in real-time on standard hardware by using efficient geometric operations and limiting the search space.

## Project Structure

```

VehiclePathPredictor/
├── main.py                    #Entry point and configuration.
├── config.py                  #Configuration constants and file paths.
├── lane\_manager.py            #LaneManager class: handles lane data.
├── vehicle\_predictor.py       #VehiclePredictor class: prediction logic.
├── tracker.py                 #LaneAwareVehicleTracker class: main processing and visualization.
├── requirements.txt           #The requirements file.
└── README.md                  #This file.

```

## Required Files
- **YOLO Model**: Pre-trained YOLO model (e.g., `yolo11m-obb.pt`)
- **Input Video**: Video file for processing
- **Lane GeoJSON**: Lane geometry data in GeoJSON format
- **Tracker Config**: YAML configuration file for object tracking (e.g., `botsort.yaml`)

## Installation

1.  Clone or download the project files.
2. Install required dependencies:
```bash
pip install -r requirements.txt
```
   

## Configuration

Update the file paths in `config.py`:

```python
MODEL_PATH = "path/to/your/yolo11m-obb.pt"
VIDEO_PATH = "path/to/your/video.mp4"
PROJECT_PATH = "path/to/your/project"
TRACKER_CONFIG = "path/to/your/botsort.yaml"
LANES_GEOJSON = "path/to/your/lanes.geojson"
````

Adjust prediction parameters in `config.py`:

```python
PAST_HISTORY_SECONDS = 4.0      #History duration for analysis.
FUTURE_PREDICTION_SECONDS = 2.0  #Prediction time horizon.
PROCESSING_FPS = 50              #Processing frame rate.
```

## Lane Data Format

The system expects lane data in GeoJSON format with the following structure:

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "properties": {
        "lane_id": "lane_1"
      },
      "geometry": {
        "type": "LineString",
        "coordinates": [[x1, y1], [x2, y2], ...]
      }
    }
  ]
}
```

## Usage

Run the system from your terminal:

```bash
python main.py
```

The system will then load the model and data, process the video, and output an annotated video with the predictions.

## Visualization and Output

The system generates a video file `CleanLanePredictor.mp4` with rich visual cues to convey predictions and confidence:

  - **Vehicle Bounding Boxes**: Detected vehicles are shown with oriented bounding boxes.
  - **Historical Paths**: Colored trails show the recent movement of each vehicle.
  - **Lane Geometry**: Gray lines show the available lanes loaded from the GeoJSON file.
  - **Predicted Paths**: The three most likely trajectories are displayed with clear color coding:
      - **Green**: The trajectory with the highest probability.
      - **Orange**: The secondary, less probable trajectory.
      - **Cyan**: The third, least probable trajectory.
  - **Confidence Styling**: The thickness of the path line varies with its probability score, providing further visual feedback on prediction confidence.
  - **Directional Arrows**: Arrowheads at the endpoints of the predicted paths clearly indicate the direction of travel.
  - **Temporal Fading**: Older predictions gradually fade out, keeping the display focused on current, relevant forecasts.
  - **Status Information**: On-screen text displays the current frame count, number of tracked vehicles, and active predictions.

## Performance Characteristics

The system typically achieves accurate predictions for vehicles traveling at moderate speeds with consistent directional movement. Prediction accuracy decreases for vehicles making sudden turns, changing lanes rapidly, or moving in areas with sparse lane coverage.

Processing speed scales roughly linearly with the number of tracked vehicles and lanes in the scene. On typical traffic videos, the system can process 20-30 vehicles simultaneously while maintaining real-time performance. GPU acceleration is highly recommended for YOLO inference to achieve these speeds.

Memory usage is controlled through the automatic cleanup of old tracking data and prediction histories, allowing the system to run continuously on long video sequences without memory leaks.

## Configuration and Customization

The system exposes numerous parameters for fine-tuning prediction behavior, which can be adjusted based on specific use cases, such as highway versus urban intersection scenarios.

  - **Core Parameters**: Adjust history duration and prediction horizon in `config.py`.
  - **Prediction Logic**: Modify alignment and distance thresholds in `vehicle_predictor.py`.
  - **Visuals**: Change visualization colors and styles in `tracker.py`.
  - **Modular Design**: The modular architecture allows for the easy replacement of individual components, such as the underlying object detector or tracking algorithm, while maintaining the core prediction logic.

## Troubleshooting

### Common Issues

  - **Missing lane file**: Ensure the GeoJSON file exists, is in the correct path, and has the correct format.
  - **No predictions**: Check if vehicles are moving consistently and if lanes are nearby and correctly mapped in the video frame.
  - **Poor performance**: Reduce video resolution or adjust processing parameters in the configuration. Ensure a GPU is being utilized if available.
  - **Tracking issues**: Verify the tracker configuration file (`botsort.yaml`) path and its contents.

### Debug Information

The system outputs progress information to the console, including the number of loaded lanes, frame processing status, active vehicle count, and prediction statistics.