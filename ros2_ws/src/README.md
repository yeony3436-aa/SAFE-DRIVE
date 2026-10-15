# ROS 2 Source Workspace

Create ROS 2 packages here only when they are needed.

Existing packages:

1. `safe_drive_bringup`
2. `safe_drive_description`
3. `safe_drive_sim`
4. `safe_drive_control`
5. `safe_drive_scenario`
6. `safe_drive_safety`

The current priority is integrating a working traffic signal, pedestrian
interaction, camera, and LiDAR into the simulation scene, then recording
repeatable validation results. Create any additional package only when its
implementation requires one. Random/Adaptive experiment code is optional
follow-up work. Avoid empty placeholder packages.
