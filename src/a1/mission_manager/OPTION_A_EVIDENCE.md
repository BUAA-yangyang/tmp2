# Option A elevator exit evidence (2026-08-04)

## Scope

This revision replaces the upper-floor dependency on an intermittently visible
in-car `DoorwayArray` landmark.  It uses the target generation's achieved
arrival heading, takes bounded MoveBase goals only through fresh known-free map
cells, and selects the corridor from perpendicular map probes.

No Gazebo/model truth enters the runtime algorithm.

## Facts supporting the design

- mf17 measured yaw preservation across the elevator teleport to about 0.3 deg.
- mf16 and mf18 produced no usable in-car doorway at the decision point.
- mf17 later froze a 1.238 m room door as the upper-floor elevator; therefore a
  generic upper-floor semantic-door freeze is not a safe control reference.
- The floor-zero wall template and `DoorwayArray` do not measure the same width:
  mf18's accepted wall-template boundary was about 1.33 m, while the intermittent
  in-car doorway observation was about 1.99 m.  This revision does not relabel
  one quantity as the other.
- Offline replay of `mf18_measured/run_0.bag` at the first frame after
  `FLOOR_SWITCH_VERIFIED` found a 1.20 m fresh known-free strip ahead of the
  arrival pose at 0.22 m half-width (`4959 / 1850258` globally known cells).
  That is sufficient for the configured first 0.85 m step plus 0.35 m forward
  footprint margin.

## Fail-closed bounds

- Unknown, occupied, out-of-grid and stale/wrong-generation maps reject a step.
- Every exit step must produce measured forward progress.
- Step count, map wait and MoveBase no-progress time are bounded.
- The corridor probe must have enough run and a measurable left/right advantage;
  ambiguous evidence stops in the lobby.
- No `start_yaw + pi`, `align_to_car_opening`, 2 m / 95 deg / 5 m fallback is in
  the active upper-floor route.

## Verification completed

- Python syntax/import through the generated devel relay.
- `catkin_make run_tests_a1_mission_manager`:
  16 tests, 0 errors, 0 failures, 0 skipped.
- Unit coverage includes unknown/occupied/out-of-grid behavior, footprint-side
  obstacles, negative grid coordinates, bounded step margins, ambiguous corridor
  probes, and structural guards on the mission state machine.

## Not yet proven

The bag replay proves availability of the first safe step only.  It does not
prove that mapping continues to reveal all later steps, that corridor selection
is unambiguous after exit, or that point-A return succeeds.  Those require a new
bounded elevator-only simulation with bag recording before any full mission run.
