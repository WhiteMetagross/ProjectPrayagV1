
## RoadMapper

A complete system for automatically generating lane geometry maps from standard traffic video footage. RoadMapper transforms raw video into structured lane data through a robust multi-stage pipeline that infers road structure from collective vehicle movement patterns.

The process begins with vehicle detection and tracking using **YOLO11-OBB** and the **BotSORT** algorithm with the VeriWild R50-Net ReID, which captures the precise orientation and path of each vehicle. The system extracts these paths as raw trajectories, which are then refined through several key algorithms:
* **Filtering & Smoothing:** Trajectories are validated based on a minimum duration to eliminate noise and smoothed using a moving average to create clean, representative paths.
* **Track Merging:** The **Hausdorff distance** algorithm identifies and consolidates trajectories from different vehicles that follow the same lane, creating a single, more accurate representation of the path.
* **Endpoint Snapping:** To form continuous lanes, the system algorithmically connects the start and end points of separate but closely aligned trajectory segments.
* **Simplification:** The resulting polylines are simplified to reduce point density while preserving the essential geometric shape of the lane.

The final output is a `lanes.geojson` file containing the generated lane polylines, alongside a visualization video overlaying the derived lanes onto the source footage for verification.

![RoadMapper Visualization](./visuals/LaneVisualisation.png)

[**See the Code/**](./RoadMapper/) | [**Read the Full Documentation.**](./RoadMapper/ReadMe.md)

---

## Vehicle Path Predictor

A lane-aware vehicle tracking and path prediction system designed to forecast future vehicle trajectories with high fidelity. The system leverages a **YOLO11 model with Oriented Bounding Boxes (OBB)** for precise vehicle detection, combined with a multi-object tracker to maintain a continuous history of vehicle positions and movements.

The core special idea lies in its **lane-constrained forecasting model**. Instead of relying on simple linear or polynomial extrapolation, the algorithm analyzes a vehicle's current velocity and direction and matches it to a pre-mapped lane network. It identifies all viable lane segments in proximity to a vehicle, calculating an alignment score based on the dot product of their direction vectors. Each potential path is assigned a probability score weighted by this alignment, the vehicle's distance to the lane centerline, and its recent movement consistency.

The system generates up to three probable future paths, each visualized with a confidence level. To ensure stable and visually coherent predictions, the output undergoes **temporal smoothing**, which averages forecasts across several frames. This methodology produces realistic, context-aware predictions that adhere to road geometry, effectively handling curves, lane changes, and vehicle occlusions.

![Vehicle Path Predictor Visualization](./visuals/TrajectoryPredictions.gif)

[**Explore the Code >**](./VehiclePathPredictorV1/) | [**Read the Full Documentation >**](./VehiclePathPredictorV1/ReadMe.md)
