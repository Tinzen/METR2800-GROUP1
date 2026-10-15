# CAD → 3D twin

How `tools/twin_model.js` was made from `METR2800 Robot.step` (Fusion 360 export).

```
npm install occt-import-js@0.0.22
pip install numpy pyfqmr
node --max-old-space-size=8000 read_step.js "METR2800 Robot.step"   # -> robot_meta.json + robot_meshes.bin
python export_model.py                                              # -> twin_model.js (copy into tools/)
```

`classify.py` assigns every mesh to a rigid body: base, arm, scoop, slide stage 0–8, bucket.
- Arm and scoop are picked by position (above the 120 mm hub, within the post width) plus four scoop mesh IDs (118, 37, 80, 119).
- Stages are picked by their offset along the slide-stack normal.
- **The mesh IDs change with every export.** After re-exporting the CAD, check the group bounding boxes it prints before trusting the twin.

Joint data the viewer uses (all in the CAD frame, mm, Z up):
- Arm pivot (0, 0, 120): 40T servo gear driving a 48T arm gear.
- Scoop pivot (0, 0, 354).
- Slide axis 32° from horizontal, 6.6° of yaw (it points at the hopper from the start-zone corner), 9 stages × 400 mm.

CAD envelope (not the real robot):
- Height 401.3 mm (bucket top).
- Footprint 393 × 370 mm (`export_model.py` drops the stray `other foot` standoff).

The height exceeds the 400 mm start-cube gauge (+0/−1 mm, R14). Measure the real robot.
