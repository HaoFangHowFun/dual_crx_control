# Physical trial 01 — left J1 latency

Analyzed the left arm (IP 192.168.2.100, controller PID 479927), 1° amplitude,
4 s period, 32 s joint-sine motion. No new motion was executed during analysis.

## Main result

The independent observer sees approximately **131 ms apparent command-to-state
phase lag**. The application sees approximately **150 ms**, because its callback
receives the same feedback messages approximately **19 ms later**.

Most of the independent lag is downstream of the host Stream Motion send boundary:
**118 ms from send to returned robot state**, stable at 117/118/119 ms in three
successive eight-second windows. This includes network transport, FANUC internal
processing, servo response and the return/status path. Host tracing cannot split
those components or prove that all 118 ms is controller-side buffering.

| Measurement | Result | Method |
| --- | ---: | --- |
| Application publish → first hardware write | median 1.070 ms; p95 1.969 ms | Exact command-vector matching |
| Application publish → observer command receipt | median 0.060 ms; p95 0.140 ms | Exact command-vector matching |
| Client enqueue → send | ~8 ms | Waveform cross-correlation |
| Driver represented command age | mean 8.217 ms | Driver's existing command timeline |
| Send call duration | mean 0.011 ms | Host timestamp brackets; not network delivery |
| Send → returned robot state | ~118 ms | Waveform phase estimate; fitted gain 0.9366 |
| Status return → first hardware read | median 0.713 ms; p95 1.403 ms | Exact full-state-vector matching |
| Observer feedback → application feedback | median 18.962 ms; p95 19.867 ms | Identical ROS header stamps, same message |
| Application command → observer feedback | ~131 ms | Waveform phase estimate |
| Application command → application feedback | ~150 ms | Waveform phase estimate |

The first report's status→hardware waveform estimate is ~4 ms. That is **not a
4 ms measured host transit**: linear interpolation between 125 Hz status samples
and the 500 Hz held state signal creates a phase/resampling difference. The exact
value-matching result above is the better host-transfer estimate. More generally,
these heterogeneous phase/host timestamp measurements should not be summed as
precise packet transit times. The 1 ms lag search resolution is not an accuracy
or confidence bound.

## Rates and feedback age

| Stage | Measured rate |
| --- | ---: |
| Diagnostic application commands | 468.2 Hz |
| Hardware write / client enqueue | 500.0 Hz |
| Stream Motion send | 125.0 Hz |
| Stream Motion status return | 125.0 Hz |
| Hardware read | 500.0 Hz |
| Passive ROS joint-state receipt | 500.0 Hz |
| Application feedback callback | 234.1 Hz |

Status sequences advanced by exactly one, and controller raw timestamp values
advanced by eight per packet. Host receipt and send intervals average 8 ms.
The raw timestamp's unit is not assumed solely from its field name. About 75%
of successive observed ROS feedback messages contain an unchanged full joint
vector, consistent with four ROS publications per new robot status.

The legacy diagnostic loop calls `spin_once` once per iteration and sleeps after
processing, so its requested 500 Hz is not its measured publication rate. It also
consumes feedback slower than the incoming 500 Hz stream. Its depth-10 feedback
queue is consistent with the observed ~19 ms receipt lag. This is a separate
application measurement issue; correcting it will not remove the ~118 ms residual
already visible in raw returned robot state.

## Other observations

- Collaborative speed scaling was exactly 1.0 in both the driver status and passive
  observer samples during the analysis. This channel shows no active scaling in
  this trial; it does not exclude other internal filtering or dynamic limits.
- Command queue depth averaged 7.999 (range 7–10). Its measured effective command
  age was ~8.2 ms; multiplying target 8 by the ROS 2 ms period would overestimate
  the observed queue/interpolation contribution in this trial.
- Returned state fitted gain was ~0.9366, roughly 6.3% lower amplitude than sent
  motion. Correlation was ~0.99995. The response is therefore not only a pure time
  shift; there is attenuation as well.
- No non-unit status sequence steps occurred in the motion interval.
- The observer started early and finished ~2.1 s before the application stopped.
  The detailed audit uses only the common steady interval, approximately 4.0–30.1 s
  after first application publication. Three eight-second windows are 4–12,
  12–20, and 20–28 s. Application/observer metadata reports no overwritten samples
  or malformed messages. Driver files span the full motion.
- Earlier ~96–100 ms results came from different Cartesian motion recordings,
  mainly measured on J2. This J1 trial is not an identical repetition; it cannot
  establish a regression from those numbers alone.

## Recommended next step

The subsequent source inspection confirmed that the sampling period is read
from a controller capability field documented in milliseconds and marked read-only.
There is no ROS launch parameter here to set it. In addition, this capture establishes a 125 Hz exchange despite
500 Hz ROS control and feedback publication. Do not assume it can be changed to
500 Hz without the appropriate controller documentation.

Then repeat this instrumented J1 baseline with only period changed to 8 s and 2 s,
keeping amplitude at 1°, buffer target and all other settings unchanged. Compare
send→status lag and fitted gain. Use at least 90 s for the passive recorder, or
start it immediately before pressing ENTER, to cover the complete trial.
The recordings can distinguish a roughly fixed residual from frequency-dependent
filtering/dynamics; they cannot by themselves expose controller execution times.
There is no evidence here that another buffer-size sweep or changing IK is the
first useful intervention.

Artifacts:

- [Standard report](trial_01_report.json): standard stage analyzer output.
- [Verified report](trial_01_verified_report.json): common-window estimates, repeated windows, exact matching,
  sample rates and duplicate-state statistics.
- [Plot](trial_01_breakdown.png): waveform, delay comparisons and packet timing.
- Local `test_results/latency/trial_01/verify_analysis.py`: trial-specific audit, requiring raw CSVs.
