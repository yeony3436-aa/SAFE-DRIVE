# Third-party visual asset inventory

The compact SAFE-Drive scene uses selected visual resources from
[osrf/car_demo](https://github.com/osrf/car_demo), commit
`38a4686dd573739dcd237187030efba3ce4913b0`. The upstream
`prius_description/package.xml` and `car_demo/package.xml` declare
`APACHE 2.0`; a copy of the license text is in
[`third-party/Apache-2.0.txt`](third-party/Apache-2.0.txt). The upstream
repository does not contain a separate license file for these individual
mesh and texture files. Preserve this attribution when redistributing them.

| Local files | Upstream files | Use |
| --- | --- | --- |
| `safe_drive_description/meshes/prius/*` | `prius_description/meshes/hybrid_body.obj`, `.mtl`, `Hybrid.png`, `Hybrid_Interior.png`, `wheel.obj`, `.mtl`, `Wheels3.png` | Visual-only Prius body and rotating wheels; body mesh scaled to `0.009 0.009 0.01` and rotated to face vehicle +x. |
| `safe_drive_sim/models/safe_drive_road/materials/textures/*` | `car_demo/models/mcity/materials/textures/Asphalt.png`, `Concrete_Ground.png`, `Grass.png` | Tiled visual surfaces on the small road, sidewalks, and verges. |

The original MCity mesh and world were inspected but are not imported. The
crosswalk paint, road collisions, signs, buildings, pedestrian model, and
vehicle collision shapes are authored in SAFE-Drive. The original ROS 1 Prius
drive plugin is not used; SAFE-Drive still drives through its ROS 2 diff-drive
baseline. Texture and mesh scale, wheel alignment, and simulation performance
should be rechecked whenever the vehicle geometry or road dimensions change.
