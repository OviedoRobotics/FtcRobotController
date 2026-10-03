# Limelight HIVE-state estimator

`LimelightCellTargeting` estimates P(audience cell up) for one alliance's HIVE and reports
`AUDIENCE_UP`, `SCORING_TABLE_UP`, or `UNKNOWN`. Odometry is responsible for aiming angle
and range. The estimator does not use odometry.

## Field geometry (game manual TU02, sections 9.6 and 9.9)

| Fact | Value | Source |
| --- | --- | --- |
| HIVE motion | Rigid, bistable seesaw on a pivot 43.95 in. above the TILES | 9.6.1, 9.6.2 |
| Tilt per stable position | 30° | Figure 9-10 |
| Tag placement | Cluster of four 3.25 in. 36h11 tags on the **bottom** of each CELL, facing the TILES | 9.9 |
| Tag orientation | Tag bottom edge toward the field center, so tag top = the CELL's outer edge | 9.9, Figure 9-16 |
| IDs | Red: 34–37 audience, 30–33 scoring table. Blue: 38–41 audience, 42–45 scoring table. IDs increase along each tag's +x | 9.9, Figure 9-17, SDK 12 `getBioBuzzCluster` |

**Hive angle** is the audience cell's outer-edge elevation:
- +30° means the audience cell is up.
- −30° means the scoring-table cell is up.

Both cells rotate together, so the scoring-table cell's tilt is the hive angle negated.

> **Correction (2026-10-03):** Earlier versions assumed that an inverted cell's tags appear
> rotated 180° in the image. A seesaw tip does not reverse the left-to-right ID order, so
> every visible cell was reported as up. That model is withdrawn.

## Visibility constraint

Tags face the floor, so from much of the field only one hive position shows tags.

- From the audience half, tags are usually visible only when the audience cell is up.
- From the far half, only when the scoring-table cell is up.
- Close to the hive, both positions are visible.

The estimator therefore has to hold its estimate through gaps in view, not just classify
frames where tags are visible.

## Per-frame measurement

1. Group detections by alliance and cell using their IDs. A cell needs at least 2 tags.
2. Fit the focal length and principal point. The fit pairs each tag's corner-mean pixel with
   the Limelight's calibrated `tx_nocross`/`ty_nocross`. All tags in the frame must have an RMS
   angular spread of at least 0.5° about their mean (for two tags, 1° apart).
3. Assign each corner to a tag-frame quadrant relative to the ID-ordered row, so the order
   of the corner list does not matter.
4. Fit the cluster jointly:
   - Initialize with a homography, then refine with a rigid 6-DOF Gauss-Newton fit.
   - Reject the fit if the RMS reprojection error exceeds 4 px, or if the face points away
     from the camera.
   - Rotation covariance = corner noise² × (JᵀJ)⁻¹. Corner noise = max(RMS residual, 0.5 px).
5. Tilt is the elevation of the tag's +y axis against the estimated world up. Its sigma is
   the fit sigma combined with a 5° mount allowance.
6. Combine both cells by inverse variance into one hive angle ± sigma.
   - Fits with a sigma above 20° are skipped.

## Estimator

Each fresh frame updates the log-odds by 0.5 × log of the likelihood ratio. The 0.5
discounts correlated frames. Log-odds are capped at ±ln 999.

The likelihood of a reading given a side (+1 audience, −1 scoring table) is a mixture:

| Component | Weight | Density |
| --- | --- | --- |
| Settled | 0.89 | Normal(±30°, sigma combined with 3°) |
| Moving | 0.10 | Uniform over that side's basin, blurred by sigma. Audience basin: −5° to +35°. Scoring basin: −35° to +5°. A bistable hive cannot cross level without flipping |
| Outlier | 0.01 | Uniform over ±90° |

Between frames and when tags are missing:

- **Unseen flips:** P relaxes toward 0.5 at 0.1 flips/s. A confident estimate stays above
  0.95 for about 0.5 s without tags.
- **Flip suspected:** Suppose tags are lost for 150 ms, and the last reading was within 22°
  of level on the side the hive last settled on, meaning it was tilting away but hadn't
  crossed. Then P moves toward the other side by up to 50% (at level).
- **Moving gate:** Two consecutive readings within 15° of level report `UNKNOWN`, whatever
  P says. The gate stays on through re-polls of the same frame and frames without tags. It
  clears on the next reading 15° or more from level, or once tags have been gone for 150 ms
  (when the flip-suspect check runs).
- **Output:** P ≥ 0.95 → `AUDIENCE_UP`; P ≤ 0.05 → `SCORING_TABLE_UP`; otherwise `UNKNOWN`.
  The threshold is a constructor parameter.

`Reason` values:

| Reason | Meaning |
| --- | --- |
| `MEASURED` | This frame updated P |
| `NO_TAGS` | Fresh frame without 2 usable tags of a cell; P held |
| `FLIP_SUSPECTED` | Tags lost mid-tilt; P leaned toward a flip |
| `UNRELIABLE` | Tags present but every fit was rejected |
| `STALE` | No fresh frame: null result, frozen timestamp, or older than 200 ms |

## Camera mount self-calibration

The configured pitch and roll are starting values. Two kinds of constraint refine them
continuously, using Newton steps on (pitch, roll) with a 15° prior on each:

| Constraint | Applies to | Information |
| --- | --- | --- |
| Up · tag x-axis = 0 (pivot axis is horizontal) | Every fitted cell, any hive position | Roll from head-on views; pitch from oblique views |
| Up · tag y-axis = ±sin 30° | Settled readings (\|angle\| ≥ 22°) while P > 0.99 or < 0.01 | Pitch, including from head-on views |

The mount estimate lives in a `LimelightCellTargeting.CameraMount`. Pass the same instance
to every estimator fed by the same camera, so red and blue cells both refine one estimate.
Each estimator adds only its own alliance's cells, so nothing is counted twice.

Call `mount.reset()` after moving the camera. The test OpMode's **triangle** button does
that and also resets the hive estimates.

## Simulated effectiveness

`tests/LimelightCellTargetingSimulation.java` runs 20 two-minute matches at 30 ms per frame.
Every input is an assumption, not a measured field condition:

| Simulation input | Value |
| --- | --- |
| Robot route | Random waypoints at 30 in./s |
| Heading | Facing the red HIVE ±10° (70% of legs), or turned up to ±120° away (30%) |
| Camera | 10 in. high; 600 px focal length; 640×480 |
| Configured pitch | 25°, against an actual 30° |
| Flips | Each takes 0.6 s; 8–28 s apart |
| Tag detection | Up to 70° off-axis and 150 in. |
| Occlusion | 10% chance per tag per frame |
| Corner noise | ±1 px |

Results are fractions of 80,000 frames. While the hive is mid-flip, either the departing
side or the arriving side counts as correct. "Measured" is the share of frames whose
reason is `MEASURED`.

| Configuration | Run with | Correct | `UNKNOWN` | Wrong | Measured |
| --- | --- | --- | --- | --- | --- |
| Fixed mount (default settings) | no argument | 62.7% | 37.2% | 0.10% | 48.5% |
| Hand-held: pitch ±13°, roll ±6° | `handheld` | 56.4% | 43.5% | 0.09% | 45.4% |

Holding the estimate turns about 14 points of frames without a measurement into correct
answers. Wrong answers come mostly from flips that happen while the camera is not looking.

The unseen-flip rate sets the hold tradeoff. These rows come from changing
`UNSEEN_FLIP_RATE` and rerunning the fixed-mount configuration:

| Unseen-flip rate | Correct | `UNKNOWN` | Wrong |
| --- | --- | --- | --- |
| 0.05/s (about a 1 s hold) | 72.2% | 27.3% | 0.46% |
| **0.1/s (about a 0.5 s hold, default)** | 62.7% | 37.2% | 0.10% |
| 0.2/s | 55.9% | 44.1% | 0.01% |
| 0.5/s | 51.3% | 48.7% | 0.00% |

The simulation has no pass/fail assertions; it is for comparing settings.

## Placeholder parameters (unmeasured)

All of these are constants in `LimelightCellTargeting.java`.

| Constant | Value | What sets it | Measure by |
| --- | --- | --- | --- |
| `SETTLED_SIGMA_DEGREES` | 3 | Hive play and field flatness | Spread of settled readings at close range |
| `MOUNT_SIGMA_DEGREES` | 5 | Residual mount error | Spread after self-calibration |
| `MOVING_PROBABILITY` | 0.10 | Fraction of frames the hive is moving | Video of match play |
| `BASIN_OVERLAP_DEGREES` | 5 | How far a bump can cross level and return | Bump the hive and record angles |
| `UNSEEN_FLIP_RATE` | 0.1/s | Flip frequency; sets the hold time | Match video; tradeoff table above |
| `FLIP_SUSPECT_DEGREES` | 22 | Angle that separates "settled" from "moving" | Bump vs flip recordings |
| `MOVING_GATE_DEGREES` | 15 | When to stop reporting a side | Flip recordings |

## Required setup

- AprilTag 36h11 pipeline with corner (`pts`) output enabled; allow IDs 30–45.
- Unmirrored image. If the camera is mounted upside down, set Limelight's image flip so
  the output image is upright.
- A rough starting camera pitch, within about 15°. Self-calibration refines it.

## Control Hub live test

`TestLimelightCellState` is listed as **Test: Limelight Cell State** (group **Test**). It
uses only the camera.

1. Connect the Limelight 3A to the Control Hub over USB and name it **limelight** in the
   active configuration.
2. Configure pipeline **0** as described above. To use another slot, change
   `APRILTAG_PIPELINE`.
3. Select the OpMode and press **INIT**. Telemetry is live in INIT and continues after Play.

Controls:
- **Dpad up/down:** change the starting pitch. This resets the estimates.
- **Triangle:** reset the estimates and the learned mount.

Telemetry per alliance:
- State and P(audience up)
- Reason, tag count and frame age
- This frame's hive angle ± sigma
- Shared by both alliances: learned pitch and roll, with the sample count

To pass:
- A settled hive reads about ±30° with a small sigma.
- P reaches the threshold within a few frames of the tags coming into view.
- With the tags out of view, P holds for about 0.5 s and then reports `UNKNOWN`.
- A hand-flipped hive reads `UNKNOWN` while moving, then the new side.
- With a fixed camera, learned pitch and roll settle within about 2° of the measured mount.

## OpMode integration

```java
Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
LimelightCellTargeting.CameraMount mount =
        new LimelightCellTargeting.CameraMount(cameraPitchDegrees, cameraRollDegrees);
LimelightCellTargeting hive = new LimelightCellTargeting(
        limelight, LimelightCellTargeting.Alliance.RED, mount);
limelight.pipelineSwitch(aprilTagPipelineIndex);
limelight.start();

// Each OpMode loop:
LimelightCellTargeting.Observation observation = hive.update();
boolean mayFeed = observation.isUp(sideSelectedByOdometry) && odometryAimReady && shooterReady;
// Apply mayFeed every loop; false must stop feeding.

// In finally/stop:
limelight.stop();
```

`sideSelectedByOdometry`, `odometryAimReady` and `shooterReady` stand for your existing
robot logic.

## Tests

`tests/LimelightCellTargetingTest.java` builds frames from a 3D model of the hive at any
angle, projects them through a pinhole camera, and parses them with the real SDK. It covers:

- convergence for every visible view and cell, with and without ±1 px noise;
- a level hive;
- holding and relaxing while out of view;
- a bump without a flip;
- flips watched from near and from far;
- turning away from a settled hive;
- duplicate, stale and disconnected frames;
- mount self-calibration from 15° off;
- corner order, missing angles, and shared red/blue frames feeding one mount.

The Android build runs neither test. To run them, put SDK 12 `Hardware` and `RobotCore`
`classes.jar` and `org.json:json:20240303` on the classpath, compile `tests/*.java` with
`LimelightCellTargeting.java`, and run `LimelightCellTargetingTest`, then
`LimelightCellTargetingSimulation`.

Hardware validation has not been done yet.

The pinhole model assumes `tx_nocross`/`ty_nocross` equal the arctangent of normalized image
coordinates on the Limelight 3A. The synthetic frames are generated with that same model,
so they cannot test the assumption. The first real capture should check it; telemetry
showing a plausible hive angle (about ±30°) is the practical check.
