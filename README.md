[![](https://jitpack.io/v/PIDPilot/ftc.svg)](https://jitpack.io/#PIDPilot/ftc)

# PIDPilot

A drop-in PIDF tuning library for FTC. Point it at a mechanism, run the OpMode, and it drives
a live FTC Dashboard tuning session — including **one-button relay auto-tune** that measures your
mechanism and computes a working starting PIDF for you, so you don't have to guess kP/kI/kD by hand.

Two tuners cover almost everything on an FTC robot:

- **`PositionPIDFTuner`** — arms, elevators, slides, turrets: anything driven to a target position.
- **`VelocityPIDFTuner`** — flywheels, intakes, any mechanism held at a target speed.

Both support DC motors; `VelocityPIDFTuner` additionally supports standard servos (open- and
closed-loop) and continuous-rotation servos with an external encoder.

## Why

Tuning PIDF gains from scratch is the single biggest barrier for new FTC teams. This library
gives you:

- **Relay auto-tune** — closes a bang-bang relay around your target, measures the resulting
  oscillation, and computes starting kP/kI/kD via Ziegler–Nichols-style formulas. Runs
  automatically the first time you start the OpMode, unless you've already supplied gains.
- **Live Dashboard tuning** — every gain, band, and timeout is a `@Config`-annotated static field,
  so you can nudge values from FTC Dashboard while the robot runs and see the effect immediately.
- **REV_UP / MAINTAIN split** — most mechanisms want different behavior when racing to a target
  (accept overshoot for speed) versus holding one under load (reject disturbances). Both tuners
  auto-tune and store both, and you flip between them live with gamepad `X`.
- **Disruption testing** — once a mechanism is holding a target, the tuner can prompt you to
  physically disturb it (push an arm, grab a flywheel) and report measured recovery time, so
  "does this hold under match conditions" is a number, not a guess.
- **Safety-aware** — soft position limits, feedforward for gravity/arm-angle, motion profiling for
  fast moves, and anti-windup are all built in and respected by both manual and auto-tuned gains.

## Install

Add JitPack, FTC Dashboard's Maven repo, and Maven Central (the FTC SDK itself is published there)
to your project's `settings.gradle.kts` (or `build.gradle` for the classic FTC sample project):

```kotlin
dependencyResolutionManagement {
    repositories {
        google()
        mavenCentral()
        maven { url = uri("https://maven.brott.dev/") } // FTC Dashboard
        maven { url = uri("https://jitpack.io") }        // PIDPilot
    }
}
```

Then add the dependency to your `TeamCode`/robot module's `build.gradle.kts`:

```kotlin
dependencies {
    implementation("com.github.PIDPilot:ftc:v1.0.4")
    implementation("com.acmerobotics.dashboard:dashboard:0.6.0")
}
```

(If you're on the classic Groovy `build.gradle`, the same coordinates work with
`implementation 'com.github.PIDPilot:ftc:v1.0.4'` syntax.) Check the
[Releases](https://github.com/PIDPilot/ftc/releases) or the badge above for the latest version tag.

## Quick start: position (arm example)

```java
@TeleOp(name = "Tune Arm", group = "Tuning")
public class TuneArm extends PIDFTunerOpMode {
    public static double TARGET_POSITION = 450.0;

    private DcMotorEx arm;

    @Override
    protected PositionPIDFTuner.Config configurePosition() {
        if (arm == null) {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
        }
        return new PositionPIDFTuner.Config()
            .target(TARGET_POSITION)
            .withMotors(arm)
            .feedforwardCosineConstant(0.12)               // arm fights gravity like cos(angle)
            .cosineFeedforwardReference(0.0, 280.0)         // ticks-at-horizontal, ticks-per-radian
            .telemetry(telemetry);
    }
}
```

Run it, press start, and leave the arm alone — the relay auto-tune phase drives the arm to
`TARGET_POSITION`, measures the induced oscillation, and computes both REV_UP and MAINTAIN gains
before handing off to normal closed-loop control. Open FTC Dashboard to watch it happen and nudge
`TARGET_POSITION` or the computed gains live. See `TuneElevator.java` for a straight-line (no
cosine) mechanism instead.

## Quick start: velocity (flywheel example)

```java
@TeleOp(name = "Tune Flywheel", group = "Tuning")
public class TuneFlywheel extends PIDFTunerOpMode {
    public static double TARGET_VELOCITY = 640.0;

    private DcMotorEx left, right;

    @Override
    protected VelocityPIDFTuner.Config configureVelocity() {
        if (left == null) {
            left = hardwareMap.get(DcMotorEx.class, "outtakeL");
            right = hardwareMap.get(DcMotorEx.class, "outtakeR");
            left.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        return new VelocityPIDFTuner.Config()
            .target(TARGET_VELOCITY)
            .withMotors(left, right)
            .averageAbsoluteVelocity(true)   // wheels spin opposite directions by design
            .telemetry(telemetry);
    }
}
```

On start, this runs a full-power characterization sweep to estimate `kF` (feedforward), settles,
then relay-auto-tunes kP/kI/kD the same way the position tuner does. See `TuneFlywheelNew.java`
for a complete example that also spins up feeder motors safely (only after `isStarted()`, never
during INIT) and runs a disruption test.

**Important:** `configureVelocity()`/`configurePosition()` are called on every control-loop
iteration (so Dashboard-edited fields like `TARGET_VELOCITY` take effect live) — cache
`hardwareMap.get(...)` results and any one-time hardware setup in fields, as shown above, instead
of repeating them every call.

## Actuator support

| Actuator | Position tuner | Velocity tuner | Relay auto-tune |
|---|---|---|---|
| DC motor | ✅ | ✅ | ✅ |
| CR servo (+ feedback encoder) | ✅ | — | ✅ |
| Standard servo, open-loop | ✅ (direct position mapping, no PID) | — | n/a — no feedback loop |
| Standard servo, closed-loop (encoder or analog feedback) | ✅ | — | not yet — tune these manually with `.revUpGains(...)`/`.maintainGains(...)` |

## Key `Config` options

Both `PositionPIDFTuner.Config` and `VelocityPIDFTuner.Config` are fluent builders. The most
commonly used methods:

| Method | Purpose |
|---|---|
| `.target(...)` | Required. Target position (ticks) or velocity (ticks/sec). |
| `.withMotors(...)` | Required (or `.withServos(...)` / `.withCRServos(...)` for position). |
| `.tuningMode(REV_UP \| MAINTAIN)` | Starting mode; toggle live with gamepad `X`. |
| `.telemetry(telemetry)` | Required. Powers both Driver Station and Dashboard telemetry. |
| `.revUpGains(kP, kI, kD, kF)` / `.maintainGains(...)` | Supply known-good gains and skip auto-tune for that mode. |
| `.skipRelayTuning()` | Disable relay auto-tune entirely; start from Dashboard-default gains. |
| `.relayAmplitude(...)` / `.relayHysteresisBandPct(...)` / `.relayDetune(...)` | Tune the auto-tune process itself if the default doesn't suit your mechanism. |
| `.runDisruptionPhase(true)` | After settling, prompt for physical disturbances and measure recovery time. |
| `.positionBounds(min, max)` *(position only)* | Soft limits — clamps targets and blocks output that would push further past a limit. |
| `.feedforwardGravityConstant(kG)` / `.feedforwardCosineConstant(kCos)` *(position only)* | Constant or angle-dependent gravity compensation. |
| `.useMotionProfile(maxVel, maxAccel)` *(position only)* | Trapezoidal profile for REV_UP moves instead of a raw step. |
| `.skipCharacterization(kF)` *(velocity only)* | Supply a known feedforward and skip the power-sweep characterization phase. |
| `.velocityRampTicksPerSecPerSec(...)` *(velocity only)* | Ramp REV_UP targets instead of stepping instantly. |

Every `Config` validates itself on start (missing target, missing hardware, conflicting actuator
types, etc.) and throws a specific `IllegalStateException` telling you exactly what to add.

## Reading the gains

- **kP** — how hard it pushes toward the target. Too low: sluggish/never arrives. Too high: overshoot/oscillation.
- **kI** — slowly corrects steady-state error (e.g., an arm sagging under a held load). Tune last, in small steps — it can mask instability that isn't actually fixed.
- **kD** — damps motion using the measurement's rate of change (derivative-on-measurement, so setpoint changes don't cause derivative kick). Raises resistance to overshoot/ringing.
- **kF** — feedforward. For velocity, it's characterized automatically (`~1/maxVelocity`). For position, it's a static-friction "kick" in direct power units — leave it at `0` unless the mechanism won't start moving from rest.
- **kG / kCos** — position-only gravity compensation: constant for an elevator/slide, `kCos·cos(angle)` for an arm.

### Symptoms and fixes

| Symptom | Likely cause |
|---|---|
| Oscillates at setpoint | `kP` too high, or `kD` too low |
| Never quite reaches target | `kP` too low, or missing `kF`/`kG`/`kCos` |
| Overshoots, then recovers | `kD` too low for the current `kP` |
| Slow on the final few ticks | `kP` too low for small errors, or needs static feedforward |
| Drifts / sags under load | Add `kG`/`kCos`, or a small `kI` |
| Oscillates right after a setpoint change | Shouldn't happen — this controller uses derivative-on-measurement specifically to avoid it. If it does, check for a very noisy measurement. |
| Integral causes overshoot | Lower `integralSumMax`, or disable `kI` in REV_UP |
| Relay auto-tune never completes | Mechanism has too much friction/backlash for `relayAmplitude` — raise it, or tune manually |

## Compatibility

Built against FTC SDK (`RobotCore`/`Hardware`) `11.2.0` and FTC Dashboard `0.6.0`. The API surface
used (`DcMotorEx`, `Servo`, `CRServo`, `AnalogInput`, `LinearOpMode`, `Telemetry`, `@Config`) has
been stable across FTC SDK seasons, so older SDK versions should work too — but match your Robot
Controller app's season if you hit a `NoSuchMethodError`.

## License

[MIT](LICENSE) © 2026 Andres Alonso. Free to use, modify, and redistribute with attribution.
