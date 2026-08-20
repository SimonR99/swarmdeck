# Known Issues & Guidelines

Active technical caveats and operational guidelines for SwarmDeck.

---

## Active Caveats

### 1. Map Registration Requires Spatial Overlap
- **Behavior**: Pairwise auto-registration (`auto` merge mode) requires at least ~35% spatial support to establish reliable loop closures.
- **Guideline**: If robots explore disjoint areas, use `static` mode with known spawn coordinates (`configs/scenarios/` or `configs/hardware_*.yaml`).

### 2. Prompt-Tuned Zero-Shot Detection Sensitivity
- **Behavior**: YOLOE-26n-seg text prompts are sensitive to phrasing (e.g. `yellow duck toy` vs generic terms).
- **Guideline**: Use the operator review drawer in the UI to confirm proposals or draw ignore zones in cluttered environments. Validate new object prompts with `tests/perception/test_catalog_recall.py`.

### 3. Aslan Hardware Domain & Extrinsics
- **Behavior**: Aslan and Botman must maintain distinct `ROS_DOMAIN_ID` settings to prevent FastDDS participant collisions.
- **Guideline**: Static transforms between camera and lidar frames must be measured precisely and passed via environment variables (e.g. `BOTMAN_OAK_X`).
