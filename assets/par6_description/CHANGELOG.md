# Changelog – Source Robotics PAR6 Description

All notable changes to this model will be documented in this file.


## [14-8-2026]
- Re-based the joint coordinates of all URDF variants (and the MJCF copies)
  onto the vendor motor convention (rcb-runtime theta): every revolute axis
  is now `+z` and the shoulder/elbow zero offsets (`-pi/2`, `pi`) are folded
  into the joint origins. The SolidWorks export's coordinates related to the
  runtime's by `theta = S*q + O`, `S = [-1,1,-1,-1,1,-1]`,
  `O = [0, -pi/2, pi, 0, 0, 0]`, while every consumer fed runtime angles in
  verbatim — self-consistent in sim, wrong against the physical arm
  (Jepson2k/par6 issue #24). Link frames, meshes and inertials are unchanged:
  the edit was verified frame-for-frame equivalent (< 1e-12) against the old
  model at mapped configurations. Config angle values (`config/PAR6.toml`)
  now apply to this model verbatim.

## [2-9-2026]
- `par6_msg_gripper` and `par6_ssg48_gripper`: the six arm joint origins now
  carry the same full-precision values as `par6_flange` (they had been
  exported with angles rounded to 1.5708 / 3.1416, skewing the J2 axis by
  3.7 µrad off perpendicular to J1). The three trees describe one arm and
  the analytic IK derivation (`par6-kin::opw`) refuses a tree that is not
  exactly OPW, which is how the rounding surfaced. Golden fixtures
  regenerated.

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

## [10-2-2025]
- Added the initial files:
    - MJCF for PAR6 with SSG48 and MSG grippers
    - URDF files for 3 versions: PAR6 with flange, SSG48 gripper and MSG gripper
    - Added extra jaw options for SSG48 gripper
