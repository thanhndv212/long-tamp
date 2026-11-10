# 0008. Gripper models: one protocol, a rotary-jaw model, a preset registry

- Status: Accepted
- Date: 2026-10-12
- Issues/PRs: #126

## Context

The grasp planner (`GraspPlanner`) and the finger closure table (`FingerClosureTable`)
were written against one class, `ParallelGripperModel`. That class assumes both pads close
symmetrically about the TCP, which holds for the Robotiq 2F-85 and the Panda hand.

The SO-101, the hardware showcase's arm (M7), has a different hand:

- one jaw swings on a revolute joint against a fixed finger;
- the midpoint between the pads moves along the closing axis as the jaw closes, from
  -2.1 mm at -10° to -19.8 mm at 30°, while the fixed pad's face stays at +2.0 mm;
- `soarm_tamp` handles this today by shifting the SRDF gripper frame by the midpoint at
  one frozen jaw angle (`geometry.GRASP_CENTERLINE_X_M`).

A symmetric model of this hand puts every grasp 10 to 20 mm off-centre. The planner then
rejects grasps that would hold, and accepts grasps whose fixed finger lands inside the
object.

Forces:

- **The existing presets must not move.** The screw-assembly batch gate and the closure
  values tested against the 2F-85 meshes depend on them.
- **More hands will come.** The roadmap names other fixed-base arms, and users bring their
  own grippers. The planner should depend on what it asks of a hand, not on one class.
- **The model is a pre-filter.** HPP's mesh collision check on the planned path stays the
  ground truth, so a coarse hull is fine. A wrong closing midpoint is not.

## Decision

1. **`GripperModel` protocol** (`long_tamp.grasping.gripper`, `runtime_checkable`). It
   holds what the planner and the closure table use, all in the canonical grasp frame
   (+X approach, +Y closing, origin at the TCP):
   - stroke:
     - `q_open`, `max_width`, `min_width`;
     - `width_at(q)`, `q_for_width(w)`;
     - `pad_x_at(q)`, `pad_x_for_width(w)`.
   - closing geometry:
     - `center_offset_for_width(w)`, where the pads' midpoint is;
     - `contact_width(y_lo, y_hi)`, the gap at which the closing pads first touch an
       object spanning `[y_lo, y_hi]`.
   - hull and joints:
     - `collision_boxes(width, sweep)`;
     - `joint_values(q, prefix)`, `open_joint_values(prefix)`.
   - attributes: `name`, `frame_rotation`, `pad_length`, `pad_width`, `default_squeeze`.

   `GraspPlanner` and `FingerClosureTable` / `GripperBinding` take any `GripperModel`.

2. **`ParallelGripperModel` implements it unchanged.** Its midpoint is 0, and
   `contact_width` is the old `2 * max(y_hi, -y_lo)`. Two optional fields, both defaulting
   to the old behaviour, describe hands whose hull is not centred:
   - `palm_offset`, the palm box's centre along the closing axis;
   - `finger_tip`, how far the fingertips reach past the pads.

3. **`RotaryJawGripperModel`**, a `ParallelGripperModel` with a per-stroke-row `center`
   table:
   - `contact_width` searches the stroke for the narrowest gap whose pads still enclose
     the object;
   - an object reaching past the fixed pad never fits;
   - the moving pad is approximated as parallel to the fixed one, and the table's gap is
     the narrowest over the pad's length, so an object that fits the model fits the hand.

4. **The planner closes about the midpoint.**
   - `evaluate` finds the open pads at `center ± max_width / 2`.
   - It reports `offset` relative to the midpoint at contact.
   - It counts pad contact against each pad's own face.
   - Sampled grasps place the midpoint, not the TCP, on the object.

5. **Presets.**
   - `PRESETS` maps a name to a model: `robotiq_2f85` and `panda_hand` are unchanged,
     `so101_jaw` is new.
   - `register_gripper(model, replace=False)` adds a model. It refuses an object that
     doesn't implement the protocol, and refuses to replace a different model of the same
     name.
   - `SO101_JAW` is measured from the URDF's meshes in the way `soarm_tamp` measures them,
     and matches its jaw table to 0.05 mm.
   - Its `<gripper>` frame is the URDF's `gripper_frame_link`, with no shifted grasp
     frame: the model carries the offset.

## Consequences

- A hand with a moving midpoint plans centred grasps. `soarm_tamp` can drop its
  frozen-angle SRDF offset once it plans with `SO101_JAW`. That change is for the M7
  hardware work and is not made here.
- Results for symmetric hands are unchanged. The 2F-85 and Panda tests pass unmodified,
  and the screw-assembly batch gate is the check.
- `RotaryJawGripperModel.contact_width` is a grid search over 2001 widths, with a
  resolution of about 0.02 mm on the SO-101, where the parallel model is exact.
- The SO-101's stroke stops at 30°. Past that, the jaw's tip swings back out of the pad
  band (the joint opens to 100°). A grasp that needs a wider opening is reported as not
  fitting.
- A hand that is neither of these (three fingers, suction) needs its own
  `GripperModel` implementation. The planner's sampling still assumes two opposing pads,
  so such a hand would also need its own planner. The protocol does not try to cover it.
