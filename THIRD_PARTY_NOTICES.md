# Third-party components

Robot meshes are downloaded on demand, not claimed as original project assets.
Keep all LICENSE, NOTICE and attribution files in `assets/upstream/` when redistributing.

- Universal Robots ROS2 Description, pinned commit `0eb1a861f7772fceb38510d90fa2f98ec668070c`: upstream licenses apply separately to code and meshes. See its retained LICENSE files.
- ManiSkill-XArm6 v0.1.1: xArm6 and Robotiq geometry; see retained upstream licenses.
- ManiSkill 3.0.1: packaged Panda geometry and agent interfaces. `robots.py` adapts the Robotiq closure constraint coordinates from ManiSkill's `XArm6Robotiq` (Apache-2.0 project). Upstream traces these coordinates to its CVPR 2022 tutorial.
- Instrument names identify reference products. Procedural approximations are not vendor CAD, endorsed digital twins, or certified operating models.
- Default instrument visuals are reconstructed from the user-provided sibling `atombridge` project's MJCF and OBJ/PNG assets: `vortex_mixer_genie_2`, `thermal_cycler_biorad_c1000`, and `centrifuge_eppendorf_5430`. Generated GLBs embed the original appearance and textures; they are local derived assets, not original Sci-Sim or manufacturer-certified CAD. Source-relative file names and SHA256 hashes are recorded in `assets/instruments/manifest.json`. No instrument redistribution license was found at that source root, so generated copies are excluded from version control; this project does not grant redistribution rights.
- The cabinet/cylindrical T-support geometry in `scene.py` and the 45-degree end-cut mesh in `stand_geometry.py` were built for this project from the user's photograph and subsequent shape corrections, with assumed dimensions documented in README. The photograph is a shape reference, not a dimensional engineering drawing.
- PCR plate and tube visuals in `assets/labware/` are local derivatives of AtomBridge container meshes, fitted to documented benchmark collision dimensions. Source paths and hashes are in `assets/labware/manifest.json`; these files are excluded from version control and carry no new redistribution permission. Racks, guide sleeves and the plate nest are procedural task fixtures.

Software dependencies keep their own licenses. This file does not grant rights beyond those licenses.
