# 4 kg pure-position-hover rollback baseline

Frozen before vertical isolation testing on 2026-08-10.  Authoritative
airframe: `px4/airframes/4027_gz_my_drone_octorotor_debug_4kg`.

```text
MPC_THR_HOVER       0.4811
MPC_Z_P             0.50
MPC_Z_VEL_P_ACC     3.00
MPC_Z_VEL_I_ACC     0.20
MPC_Z_VEL_D_ACC     0.20
MPC_Z_VEL_MAX_UP    0.40
MPC_Z_VEL_MAX_DN    0.30
```

Isolation contract for every run in this series:

```text
ENABLE_ARM_CONTROL=false
ARM_FEEDFORWARD_ENABLED=false
ARM_TORQUE_FEEDFORWARD_ENABLED=false
ARM_STATIC_COM_FEEDFORWARD_GAIN=0.0
BATTERY_DYNAMICS_ENABLED=false
ENABLE_SENSOR_DELAY=true   # relay required; all four delay values are 0 ms
No H, R/F, WASD, yaw, or arm input
T -> measured steady hover for 30 s -> L
```

The 4 kg physical profile reports `px4_hover_command=0.4811252243`, total
mass 4.0 kg and ideal-linear thrust mapping.  Parameter experiments must be
changed one stage at a time and rejected values must be restored here.

## Accepted isolated-hover tuning (2026-08-10)

After one baseline run and sequential A/B experiments, the reproducible
30-second pure-position-hover setting is:

```text
MPC_THR_HOVER       0.4811
MPC_Z_P             0.35
MPC_Z_VEL_P_ACC     2.20
MPC_Z_VEL_I_ACC     0.20
MPC_Z_VEL_D_ACC     0.20
MPC_Z_VEL_MAX_UP    0.25
MPC_Z_VEL_MAX_DN    0.25
```

Accepted evidence:

| run | PX4 height p-p | Gazebo truth p-p | PX4/truth vz p90 | motor mean | result |
|---|---:|---:|---:|---:|---|
| `hover_4kg_z_trialB_zp035_vp220_20260810.log` | 0.118 m | 0.141 m | 0.0182 / 0.0415 m/s | 478.59 | PASS |
| `hover_4kg_z_trialB_zp035_vp220_repeat_20260810.log` | 0.134 m | 0.134 m | 0.0735 / 0.0685 m/s | 482.29 | PASS |

Both runs had zero motor saturation, no failsafe, and confirmed landing
disarm.  Trial C was not run because trial B passed twice.  The original
values at the top of this file remain the rollback baseline.

## Restored operator controls and arm ladder (2026-08-10)

R/F/H was restored only after the two isolated-hover passes.  The PX4 safety
limits remain 0.25 m/s in both vertical directions; the 4 kg launcher now uses
a gentler `PX4_WASD_VERTICAL_SPEED_M_S=0.15` manual command because 0.25 m/s
produced a repeatable descent overshoot up to 0.469 m/s.

| run | scope | max horizontal speed | max vertical speed | saturation | result |
|---|---|---:|---:|---:|---|
| `velocity_rf_h_4kg_v015_20260810.log` | R, release, F, release, R interrupted by H | 0.070 m/s | 0.254 m/s | 0 | PASS |
| `velocity_rf_h_4kg_v015_repeat_20260810.log` | clean-start repeat | 0.069 m/s | 0.247 m/s | 0 | PASS |

The test waits for three continuous seconds at captured altitude with
`abs(vz) < 0.08 m/s` before sending any key.  Both runs confirmed H returning
the controller to position hold, no failsafe, and landing/disarm.

The arm was then restored with all feedforward paths still disabled:

| run | arm motion | horizontal drift | altitude span | max arm torque | saturation | result |
|---|---|---:|---:|---:|---:|---|
| `arm_micro_4kg_retry_20260810.log` | micro A -> micro B -> retracted | 0.120 m | 0.616 m | 0.037 N m | 0 | PASS |
| `arm_key6_4kg_20260810.log` | visible key-6 extended -> retracted cycle | 0.123 m | 0.251 m | 0.029 N m | 0 | PASS |

The micro run's relatively large altitude span includes isolated vertical
estimator/response spikes and is retained as a warning, not hidden.  The
operator-visible key-6 cycle completed without the previously reported drop,
without motor saturation or failsafe, and landed/disarmed normally.
