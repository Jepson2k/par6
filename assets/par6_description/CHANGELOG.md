# Changelog – Source Robotics PAR6 Description

All notable changes to this model will be documented in this file.


## [13-8-2026]
- Replaced the moving-link inertials (`shoulder` … `wrist`) in all three URDF
  variants with the vendor runtime's dynamics table (Source Robotics
  rcb-runtime `robots/PAR6.py`), re-expressed in each link's frame. The
  SolidWorks-exported values modeled only the printed shell: 2.375 kg of
  moving mass against the vendor's 5.114 kg (~2.2x light), which under-drove
  gravity compensation and the torque-level simulator plant.
- `par6_flange` variant: the `gripper` (flange plate) link now carries the
  vendor Flange tool entry (0.0555 kg at the tool origin); its rotational
  inertia keeps the SolidWorks tensor (the vendor table carries none, G(q)
  never reads it, and the sim plant's forward dynamics needs a nonsingular
  wrist inertia).
- Added `par6_flange/urdf/par6_arm.urdf`: the arm-only gravity chain (the
  flange URDF with a massless tool stub). The runtime attaches the active
  tool's inertials from the gripper config instead, so tool mass has exactly
  one source. MSG/SSG48 tool-side links keep their CAD inertials — they are
  visual/self-consistency data, not the gravity source.
- Reference check: `crates/par6-kin/tests/golden/gravity/vendor_reference.json`
  pins G(q) on these URDFs to the vendor table independently of any URDF.

## [10-2-2025]
- Added the initial files:
    - MJCF for PAR6 with SSG48 and MSG grippers
    - URDF files for 3 versions: PAR6 with flange, SSG48 gripper and MSG gripper
    - Added extra jaw options for SSG48 gripper
