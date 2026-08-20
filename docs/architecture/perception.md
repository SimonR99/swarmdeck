# Perception & Object Detection

SwarmDeck incorporates an open-vocabulary object detection and 3D spatial projection pipeline to identify targets (such as rubber ducks, cones, and obstacles) and project them into the global map.

---

## 1. Detection Architecture

```text
┌─────────────────┐       ┌────────────────────────┐       ┌──────────────────────┐
│  Robot Camera   │ RGB   │ YOLOE Sidecar Server   │ BBoxes│ Depth Projection     │
│ (OAK-D/RealSense├──────►│ (YOLOE-26n-seg on GPU) ├──────►│ (RGBD De-projection) │
└─────────────────┘       └────────────────────────┘       └──────────┬───────────┘
                                                                      │ (X, Y, Z) in Map
                                                           ┌──────────▼───────────┐
                                                           │ SwarmDeck Backend    │
                                                           │ (Review & Map Store) │
                                                           └──────────────────────┘
```

1. **Inference Sidecar** (`adapters/perception/yoloe_server.py`):
   - Runs open-vocabulary promptable segmentation (`yoloe-26n-seg.pt`).
   - Accepts prompt text lists and returns 2D bounding boxes, polygon masks, class labels, and confidence scores.
2. **Catalog & Confidence Floors** (`adapters/perception/catalog.py`):
   - Defines target classes with empirically verified prompt embeddings and per-class minimum confidence floors.
   - Verified via reference photographs in `tests/perception/fixtures/`.
3. **3D Depth Projection** (`adapters/perception/depth_projection.py`):
   - Aligns RGB detection masks with depth/disparity maps.
   - Samples depth within the detected mask, removes floor/background outliers, and projects centroid coordinates into 3D robot frame.
   - Transforms 3D target coordinates into the global map frame via TF / odometry.

---

## 2. Operator Review & Persistence

Detections are presented to the operator in the web UI for validation:
- **Candidate Proposal**: A newly detected object appears on the map as a tentative marker.
- **Operator Actions**:
  - **Confirm**: Locks the detection as a verified map landmark.
  - **Reject**: Dismisses false positives.
  - **Ignore Zone**: Creates an exclusion polygon where further detections are ignored.
- **Persistence**: Approved and rejected detections persist in `sessions/detections.json` across server restarts.
