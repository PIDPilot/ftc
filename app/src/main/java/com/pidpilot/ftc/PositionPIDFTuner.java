package com.pidpilot.ftc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

@Config
public class PositionPIDFTuner {
    /** Default aggressive proportional gain shown in Dashboard. */
    public static double REV_UP_KP = 0.9;
    /** Default aggressive integral gain shown in Dashboard; REV_UP still disables integral in code. */
    public static double REV_UP_KI = 0.0;
    /** Default aggressive derivative gain shown in Dashboard. */
    public static double REV_UP_KD = 0.12;
    /** Default aggressive static-trim gain in direct actuator-power units shown in Dashboard. */
    public static double REV_UP_KF = 0.0;
    /** Default hold proportional gain shown in Dashboard. */
    public static double MAINTAIN_KP = 0.6;
    /** Default hold integral gain shown in Dashboard. */
    public static double MAINTAIN_KI = 0.03;
    /** Default hold derivative gain shown in Dashboard. */
    public static double MAINTAIN_KD = 0.08;
    /** Default hold static-trim gain in direct actuator-power units shown in Dashboard. */
    public static double MAINTAIN_KF = 0.0;
    /** Default integral cap in motor-power units. */
    public static double DEFAULT_INTEGRAL_SUM_MAX = 0.25;
    /** Default derivative low-pass alpha where lower values smooth more. */
    public static double DEFAULT_DERIVATIVE_ALPHA = 0.2;

    /** Tiny threshold used for floating-point comparisons. */
    private static final double EPSILON = 1e-6;
    /** Smallest allowed absolute scale for normalized position control. */
    private static final double MIN_NORMALIZATION_TICKS = 1.0;
    /** Scale floor multiplier so tiny moves do not make the normalized loop too aggressive. */
    private static final double TOLERANCE_SCALE_MULTIPLIER = 10.0;
    /** FTC motor power hard limit. */
    private static final double MAX_POWER = 1.0;
    /** Servo position range hard limits. */
    private static final double MIN_SERVO_POSITION = 0.0;
    private static final double MAX_SERVO_POSITION = 1.0;
    /** Unit conversion from seconds to milliseconds. */
    private static final double MILLIS_PER_SECOND = 1000.0;
    /** Consecutive loops required before a position is declared stable. */
    private static final int REQUIRED_AT_TARGET_LOOPS = 5;
    /** Relay auto-tune bang-bang power magnitude applied while measuring the oscillation. */
    private static final double DEFAULT_RELAY_AMPLITUDE = 0.3;
    private static final double MIN_RELAY_AMPLITUDE = 0.05;
    private static final double MAX_RELAY_AMPLITUDE = 0.7;
    /** Hysteresis band around the target, as a fraction of |target|, before the relay switches sign. */
    private static final double DEFAULT_RELAY_HYSTERESIS_BAND_PCT = 0.03;
    private static final double DEFAULT_RELAY_DETUNE = 1.0;
    private static final double MIN_RELAY_DETUNE = 0.1;
    private static final double MAX_RELAY_DETUNE = 2.0;
    private static final int RELAY_REQUIRED_CYCLES = 4;
    private static final double RELAY_MAX_OSCILLATION_SECONDS = 15.0;
    private static final double RELAY_COMPLETE_HOLD_SECONDS = 1.0;

    private final PIDFController controller = new PIDFController(0.0, 0.0, 0.0, 0.0);
    private final Telemetry driverTelemetry;
    private final Telemetry dashboardTelemetry;
    private final DisruptionPhase disruptionPhase = new DisruptionPhase();
    private final RelayAutoTuner relayAutoTuner = new RelayAutoTuner();

    private ActuatorMode actuatorMode;
    private ServoFeedbackMode servoFeedbackMode;
    private DcMotorEx[] motors;
    private Servo[] standardServos;
    private CRServo[] crServos;
    private DcMotorEx[] feedbackEncoders;
    private AnalogInput servoFeedbackAnalogInput;
    private GainSet revUpGains;
    private GainSet maintainGains;
    private PIDFTuningMode mode;
    private double integralSumMax;
    private double derivativeAlpha;
    private double requestedTargetTicks;
    private double unclampedRequestedTargetTicks;
    private double profiledTargetTicks;
    private double profiledVelocityTicksPerSecond;
    private double profiledAccelerationTicksPerSecondSquared;
    private boolean motionProfileEnabled;
    private double maxProfileVelocity;
    private double maxProfileAcceleration;
    private double positionToleranceTicks;
    private double gravityFeedforwardConstant;
    private double cosineFeedforwardConstant;
    private double cosineZeroTicks;
    private double cosineTicksPerRadian;
    private double servoOutputScale;
    private double servoRangeMinTicks;
    private double servoRangeMaxTicks;
    private double servoRangeTicksPerUnit;
    private double analogVoltageToTicksScale;
    private boolean positionConstraintsConfigured;
    private double minPositionTicks;
    private double maxPositionTicks;
    private boolean runDisruptionPhase;
    private int disruptionSamples;
    private long disruptionReadyStableMs;
    private long disruptionDetectTimeoutMs;
    private long disruptionRecoveryTimeoutMs;
    private double disruptionReadyBandPct;
    private double disruptionDropThresholdPct;
    private boolean manualRevUpConfigured;
    private boolean manualMaintainConfigured;
    private boolean skipRelayTuning;
    private double relayAmplitude;
    private double relayHysteresisBandPct;
    private double relayDetune;
    private GainSet relayComputedRevUpGains;
    private GainSet relayComputedMaintainGains;
    private String relayTuneNote = "";
    private TunerPhase tunerPhase = TunerPhase.RUNNING;
    private boolean relayTuneEvaluated;
    private long phaseStartNs;

    private double averagePositionTicks;
    private double moveScaleTicks = MIN_NORMALIZATION_TICKS;
    private double profileStartTicks;
    private double profileDistanceTicks;
    private double profileElapsedSeconds;
    private double lastRequestedTargetTicks = Double.NaN;
    private double lastOutput;
    private double lastFinalError;
    private double lastLoopTimeSeconds = PIDFTunerOpMode.DEFAULT_LOOP_TIME_SECONDS;
    private double lastFeedforwardTerm;
    private double lastStaticTrimTerm;
    private double lastGravityFeedforwardTerm;
    private double lastCosineFeedforwardTerm;
    private double lastActuatorCommand;
    private int atTargetLoopCount;
    private String positionConstraintStatus = "none";

    public PositionPIDFTuner(Config config) {
        config.validate();
        driverTelemetry = config.telemetry;
        dashboardTelemetry = PIDFTunerOpMode.getDashboardTelemetry();
        refreshFrom(config, config.getResolvedMode());
    }

    public void refreshFrom(Config config, PIDFTuningMode forcedMode) {
        config.validate();
        ActuatorMode newActuatorMode = config.resolveActuatorMode();
        boolean actuatorChanged = newActuatorMode != actuatorMode;
        actuatorMode = newActuatorMode;
        servoFeedbackMode = config.resolveServoFeedbackMode();
        motors = config.motors;
        standardServos = config.standardServos;
        crServos = config.crServos;
        feedbackEncoders = config.resolveFeedbackEncoders();
        servoFeedbackAnalogInput = config.servoFeedbackAnalogInput;
        manualRevUpConfigured = config.hasManualRevUpGains();
        manualMaintainConfigured = config.hasManualMaintainGains();
        revUpGains = resolveRevUpGains(config);
        maintainGains = resolveMaintainGains(config);
        integralSumMax = config.resolveIntegralSumMax();
        derivativeAlpha = config.resolveDerivativeAlpha();
        unclampedRequestedTargetTicks = config.targetTicks;
        positionConstraintsConfigured = config.positionConstraintsConfigured;
        minPositionTicks = config.minPositionTicks;
        maxPositionTicks = config.maxPositionTicks;
        requestedTargetTicks = constrainTargetTicks(config.targetTicks);
        motionProfileEnabled = config.motionProfileEnabled;
        maxProfileVelocity = config.maxProfileVelocity;
        maxProfileAcceleration = config.maxProfileAcceleration;
        positionToleranceTicks = config.positionToleranceTicks;
        gravityFeedforwardConstant = config.feedforwardGravityConstant;
        cosineFeedforwardConstant = config.feedforwardCosineConstant;
        cosineZeroTicks = config.cosineZeroTicks;
        cosineTicksPerRadian = config.cosineTicksPerRadian;
        servoOutputScale = config.servoOutputScale;
        servoRangeMinTicks = config.servoRangeMinTicks;
        servoRangeMaxTicks = config.servoRangeMaxTicks;
        servoRangeTicksPerUnit = servoRangeMaxTicks - servoRangeMinTicks;
        analogVoltageToTicksScale = config.analogVoltageToTicksScale;
        runDisruptionPhase = config.runDisruptionPhase;
        disruptionSamples = config.disruptionSamples;
        disruptionReadyStableMs = config.disruptionReadyStableMs;
        disruptionDetectTimeoutMs = config.disruptionDetectTimeoutMs;
        disruptionRecoveryTimeoutMs = config.disruptionRecoveryTimeoutMs;
        disruptionReadyBandPct = config.disruptionReadyBandPct;
        disruptionDropThresholdPct = config.disruptionDropThresholdPct;
        skipRelayTuning = config.skipRelayTuning;
        relayAmplitude = config.relayAmplitude;
        relayHysteresisBandPct = config.relayHysteresisBandPct;
        relayDetune = config.relayDetune;
        if (actuatorChanged) {
            prepareConfiguredHardware();
            controller.reset();
            disruptionPhase.reset();
            atTargetLoopCount = 0;
            lastRequestedTargetTicks = Double.NaN;
            relayTuneEvaluated = false;
        }
        setMode(forcedMode == null ? config.getResolvedMode() : forcedMode);
        if (Double.isNaN(lastRequestedTargetTicks)) {
            averagePositionTicks = readPositionMeasurement();
            profiledTargetTicks = requestedTargetTicks;
        }
        syncRelayTuneMode();
    }

    private GainSet resolveRevUpGains(Config config) {
        if (config.hasManualRevUpGains()) return config.revUpGains;
        return relayComputedRevUpGains != null ? relayComputedRevUpGains : config.resolveRevUpGains();
    }

    private GainSet resolveMaintainGains(Config config) {
        if (config.hasManualMaintainGains()) return config.maintainGains;
        return relayComputedMaintainGains != null ? relayComputedMaintainGains : config.resolveMaintainGains();
    }

    private boolean shouldRunRelayTuning() {
        return !skipRelayTuning && !(manualMaintainConfigured && manualRevUpConfigured);
    }

    private void syncRelayTuneMode() {
        if (relayTuneEvaluated) {
            return;
        }
        relayTuneEvaluated = true;
        boolean relaySupported = actuatorMode == ActuatorMode.MOTOR || actuatorMode == ActuatorMode.CR_SERVO;
        if (!relaySupported) {
            tunerPhase = TunerPhase.RUNNING;
            relayTuneNote = "Relay auto-tune only supports MOTOR and CR_SERVO actuators; configure gains manually for servo position control.";
            return;
        }
        if (shouldRunRelayTuning()) {
            startRelayTuning();
        } else {
            tunerPhase = TunerPhase.RUNNING;
            relayTuneNote = resolveRelaySkipNote();
        }
    }

    private void startRelayTuning() {
        tunerPhase = TunerPhase.RELAY_TUNING;
        phaseStartNs = System.nanoTime();
        relayAutoTuner.reset();
        relayTuneNote = "";
        controller.reset();
        disruptionPhase.reset();
        atTargetLoopCount = 0;
    }

    private String resolveRelaySkipNote() {
        if (skipRelayTuning) return "Skipping relay auto-tune: skipRelayTuning() configured.";
        if (manualMaintainConfigured && manualRevUpConfigured) return "Skipping relay auto-tune: manual gains configured.";
        return "";
    }

    private void appendRelayTuneNote(String message) {
        if (message == null || message.isEmpty()) return;
        relayTuneNote = relayTuneNote.isEmpty() ? message : relayTuneNote + " | " + message;
    }

    private void applyRelayComputedGains() {
        GainSet computedMaintain = relayAutoTuner.getComputedMaintainGains();
        GainSet computedRevUp = relayAutoTuner.getComputedRevUpGains();
        if (computedMaintain == null || computedRevUp == null) return;
        relayComputedMaintainGains = computedMaintain;
        relayComputedRevUpGains = computedRevUp;
        if (!manualMaintainConfigured) maintainGains = relayComputedMaintainGains;
        if (!manualRevUpConfigured) revUpGains = relayComputedRevUpGains;
    }

    /**
     * The {@code moveScaleTicks} the RUNNING phase normalizes error by when the mechanism is sitting
     * on target (distance term ~0), which is exactly the situation right after relay auto-tune —
     * {@link #updateProfile} computes this same value on the first RUNNING loop. The relay measures
     * Ku/Pu in raw ticks, so the computed gains must be scaled by this factor to survive the
     * controller's error normalization and land at the intended strength.
     */
    private double resolveHoldMoveScaleTicks() {
        return Math.max(MIN_NORMALIZATION_TICKS, positionToleranceTicks * TOLERANCE_SCALE_MULTIPLIER);
    }

    private double getPhaseElapsedSeconds() {
        if (phaseStartNs == 0L) return 0.0;
        return (System.nanoTime() - phaseStartNs) * 1e-9;
    }

    public void setMode(PIDFTuningMode newMode) {
        PIDFTuningMode resolvedMode = newMode == null ? PIDFTuningMode.MAINTAIN : newMode;
        if (resolvedMode != mode) {
            mode = resolvedMode;
            controller.reset();
            disruptionPhase.reset();
            atTargetLoopCount = 0;
            lastRequestedTargetTicks = Double.NaN;
        }
        applyActiveGains();
    }

    public PIDFTuningMode getMode() {
        return mode;
    }

    public boolean isAtTarget() {
        return atTargetLoopCount >= REQUIRED_AT_TARGET_LOOPS;
    }

    /**
     * Call once, immediately after {@code waitForStart()} returns. The constructor (and therefore
     * relay auto-tune, which starts immediately if enabled) runs during INIT, before the driver
     * presses start — INIT dwell is routinely tens of seconds in real competition, and without this
     * call that entire wait would count against the relay oscillation time budget, causing it to
     * time out instantly the moment the match starts instead of actually measuring the mechanism.
     * {@link PIDFTunerOpMode} calls this automatically; only call it directly if you are driving
     * this tuner from your own OpMode loop instead.
     */
    public void onStart() {
        if (tunerPhase == TunerPhase.RELAY_TUNING) {
            phaseStartNs = System.nanoTime();
            relayAutoTuner.reset();
        }
    }

    public void update(double loopTimeSeconds) {
        lastLoopTimeSeconds = loopTimeSeconds;

        if (actuatorMode == ActuatorMode.STANDARD_SERVO_OPEN_LOOP) {
            runStandardServoOpenLoop();
            return;
        }

        averagePositionTicks = readPositionMeasurement();
        lastFinalError = requestedTargetTicks - averagePositionTicks;

        if (tunerPhase == TunerPhase.RELAY_TUNING) {
            runRelayTuningLoop();
            return;
        }
        if (tunerPhase == TunerPhase.RELAY_COMPLETE) {
            runRelayCompleteLoop();
            return;
        }

        updateProfile(loopTimeSeconds);
        applyActiveGains();

        // POSITION PIDF OUTPUT ASSEMBLY
        // output = pTerm + iTerm + dTerm + fTerm + gravityTerm
        //
        // pTerm       = kP * error
        // iTerm       = kI * integralSum              (anti-windup clamped)
        // dTerm       = -kD * filteredDMeasurement    (derivative on measurement)
        // fTerm       = kF * sign(error)              (stiction kick — set kF=0 if not needed)
        // gravityTerm = kGravity                      (elevator) OR kCos*cos(angle) (arm)
        //
        // NOTE: kF here is NOT a velocity feedforward. Do not set it to 1/maxVelocity.
        // The velocity tuner has its own kF characterization phase for that purpose.
        double normalizedTarget = profiledTargetTicks / moveScaleTicks;
        double normalizedMeasurement = averagePositionTicks / moveScaleTicks;
        double pidOutput = controller.calculate(normalizedTarget, normalizedMeasurement, loopTimeSeconds);
        lastFeedforwardTerm = computeNormalizedFeedforward();

        switch (actuatorMode) {
            case MOTOR:
                lastOutput = clip(constrainClosedLoopOutput(pidOutput + lastFeedforwardTerm), -MAX_POWER, MAX_POWER);
                lastActuatorCommand = lastOutput;
                applyMotorPower(lastOutput);
                break;
            case CR_SERVO:
                lastOutput = clip(constrainClosedLoopOutput(pidOutput + lastFeedforwardTerm), -MAX_POWER, MAX_POWER);
                lastActuatorCommand = clip(lastOutput * servoOutputScale, -MAX_POWER, MAX_POWER);
                applyCRServoPower(lastActuatorCommand);
                break;
            case STANDARD_SERVO_CLOSED_LOOP:
                double baseServoPosition = mapTicksToServoPosition(profiledTargetTicks);
                double servoCorrection = constrainClosedLoopOutput(pidOutput + lastFeedforwardTerm);
                lastOutput = clip((baseServoPosition + servoCorrection) * servoOutputScale, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
                lastActuatorCommand = lastOutput;
                applyStandardServoPosition(lastOutput);
                break;
            default:
                throw new IllegalStateException("Unsupported actuator mode: " + actuatorMode);
        }

        updateAtTargetCounter();
        updateDisruptionPhase();
        pushTelemetry();
    }

    public void pushFinalSummary() {
        String summary = disruptionPhase.summary(actuatorMode == ActuatorMode.STANDARD_SERVO_OPEN_LOOP);
        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, "Final position summary");
        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, summary);
        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry,
            String.format(Locale.US, "Final MAINTAIN PIDF: kP=%.6f kI=%.6f kD=%.6f",
                maintainGains.kP, maintainGains.kI, maintainGains.kD));
        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry,
            String.format(Locale.US, "Final REV_UP PIDF:   kP=%.6f kI=%.6f kD=%.6f",
                revUpGains.kP, revUpGains.kI, revUpGains.kD));
        PIDFTunerOpMode.updateTelemetry(driverTelemetry, dashboardTelemetry);
    }

    /**
     * Drives the bang-bang relay directly (bypassing the user's PID gains) while gravity/cosine
     * feedforward keeps applying, so the mechanism oscillates around the target instead of just
     * sagging under load. Unlike VelocityPIDFTuner's relay phase, there is no separate
     * "approach target" step: closing the relay loop on position error already drives the
     * mechanism to the target and induces the limit cycle in one motion.
     */
    private void runRelayTuningLoop() {
        profiledTargetTicks = requestedTargetTicks;
        lastFeedforwardTerm = computeNormalizedFeedforward();
        double relayTerm = relayAutoTuner.update(lastFinalError);
        double rawOutput = constrainClosedLoopOutput(lastFeedforwardTerm + relayTerm);
        lastOutput = clip(rawOutput, -MAX_POWER, MAX_POWER);
        applyRelayActuatorOutput();
        if (relayAutoTuner.hasCompletedSuccessfully() || relayAutoTuner.shouldSkipToRunning()) {
            applyRelayComputedGains();
            appendRelayTuneNote(relayAutoTuner.getCompletionMessage());
            tunerPhase = relayAutoTuner.hasCompletedSuccessfully() ? TunerPhase.RELAY_COMPLETE : TunerPhase.RUNNING;
            phaseStartNs = System.nanoTime();
            controller.reset();
            atTargetLoopCount = 0;
            lastRequestedTargetTicks = Double.NaN;
        }
        updateAtTargetCounter();
        pushTelemetry();
    }

    private void runRelayCompleteLoop() {
        profiledTargetTicks = requestedTargetTicks;
        lastFeedforwardTerm = computeNormalizedFeedforward();
        lastOutput = clip(constrainClosedLoopOutput(lastFeedforwardTerm), -MAX_POWER, MAX_POWER);
        applyRelayActuatorOutput();
        if (getPhaseElapsedSeconds() >= RELAY_COMPLETE_HOLD_SECONDS) {
            tunerPhase = TunerPhase.RUNNING;
            controller.reset();
            atTargetLoopCount = 0;
            // NaN forces the first RUNNING loop to reinitialize the motion profile from the current
            // (on-target) position AND recompute moveScaleTicks to resolveHoldMoveScaleTicks() — the
            // exact scale computeGains() used — so the relay gains land at the intended strength.
            lastRequestedTargetTicks = Double.NaN;
        }
        pushTelemetry();
    }

    private void applyRelayActuatorOutput() {
        if (actuatorMode == ActuatorMode.CR_SERVO) {
            lastActuatorCommand = clip(lastOutput * servoOutputScale, -MAX_POWER, MAX_POWER);
            applyCRServoPower(lastActuatorCommand);
        } else {
            lastActuatorCommand = lastOutput;
            applyMotorPower(lastOutput);
        }
    }

    private void runStandardServoOpenLoop() {
        controller.reset();
        profiledTargetTicks = requestedTargetTicks;
        profiledVelocityTicksPerSecond = 0.0;
        profiledAccelerationTicksPerSecondSquared = 0.0;
        averagePositionTicks = requestedTargetTicks;
        lastFinalError = 0.0;
        lastFeedforwardTerm = 0.0;
        lastOutput = clip(mapTicksToServoPosition(requestedTargetTicks) * servoOutputScale, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
        lastActuatorCommand = lastOutput;
        applyStandardServoPosition(lastOutput);
        atTargetLoopCount = REQUIRED_AT_TARGET_LOOPS;
        disruptionPhase.reset();
        pushTelemetry();
    }

    private void prepareConfiguredHardware() {
        if (motors != null) {
            for (DcMotorEx motor : motors) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(0.0);
            }
        }
        if (feedbackEncoders != null) {
            for (DcMotorEx encoder : feedbackEncoders) {
                encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                encoder.setPower(0.0);
            }
        }
        if (crServos != null) {
            for (CRServo servo : crServos) {
                servo.setPower(0.0);
            }
        }
    }

    private void applyActiveGains() {
        GainSet gains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
        controller.setGains(gains.kP, mode == PIDFTuningMode.REV_UP ? 0.0 : gains.kI, gains.kD, 0.0);
        controller.integralSumMax = Math.abs(controller.getKI()) > EPSILON
            ? integralSumMax / Math.abs(controller.getKI())
            : integralSumMax;
        controller.derivativeAlpha = derivativeAlpha;
    }

    private double readPositionMeasurement() {
        switch (actuatorMode) {
            case MOTOR:
                return averageMotorPositions(motors);
            case STANDARD_SERVO_CLOSED_LOOP:
                if (servoFeedbackMode == ServoFeedbackMode.MOTOR_ENCODER) {
                    return averageMotorPositions(feedbackEncoders);
                }
                return servoFeedbackAnalogInput.getVoltage() * analogVoltageToTicksScale;
            case CR_SERVO:
                return averageMotorPositions(feedbackEncoders);
            case STANDARD_SERVO_OPEN_LOOP:
                return requestedTargetTicks;
            default:
                return 0.0;
        }
    }

    private void updateProfile(double loopTimeSeconds) {
        boolean targetChanged = Double.isNaN(lastRequestedTargetTicks)
            || Math.abs(requestedTargetTicks - lastRequestedTargetTicks) > EPSILON;
        if (targetChanged) {
            profileStartTicks = averagePositionTicks;
            profileDistanceTicks = requestedTargetTicks - averagePositionTicks;
            profileElapsedSeconds = 0.0;
            lastRequestedTargetTicks = requestedTargetTicks;
            moveScaleTicks = Math.max(
                MIN_NORMALIZATION_TICKS,
                Math.max(Math.abs(profileDistanceTicks), positionToleranceTicks * TOLERANCE_SCALE_MULTIPLIER)
            );
        }

        if (mode == PIDFTuningMode.REV_UP && motionProfileEnabled) {
            profileElapsedSeconds += loopTimeSeconds;
            MotionState state = calculateMotionState(profileDistanceTicks, profileElapsedSeconds);
            profiledTargetTicks = profileStartTicks + state.positionTicks;
            profiledVelocityTicksPerSecond = state.velocityTicksPerSecond;
            profiledAccelerationTicksPerSecondSquared = state.accelerationTicksPerSecondSquared;
        } else {
            profiledTargetTicks = requestedTargetTicks;
            profiledVelocityTicksPerSecond = 0.0;
            profiledAccelerationTicksPerSecondSquared = 0.0;
        }
    }

    private double computeNormalizedFeedforward() {
        lastStaticTrimTerm = computeStaticTrimTerm();
        lastGravityFeedforwardTerm = gravityFeedforwardConstant;
        lastCosineFeedforwardTerm = cosineFeedforwardConstant == 0.0
            ? 0.0
            : cosineFeedforwardConstant * Math.cos((averagePositionTicks - cosineZeroTicks) / cosineTicksPerRadian);
        return lastStaticTrimTerm + lastGravityFeedforwardTerm + lastCosineFeedforwardTerm;
    }

    private double computeStaticTrimTerm() {
        GainSet gains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
        if (Math.abs(lastFinalError) <= positionToleranceTicks) {
            return 0.0;
        }
        // Position-control kF is a static friction kick, not a velocity feedforward.
        // Set it to the minimum power that just starts the mechanism moving.
        return gains.kF * Math.signum(lastFinalError);
    }

    private double resolveActivePositionKf() {
        GainSet gains = mode == PIDFTuningMode.MAINTAIN ? maintainGains : revUpGains;
        return gains.kF;
    }

    private void updateAtTargetCounter() {
        if (actuatorMode == ActuatorMode.STANDARD_SERVO_OPEN_LOOP) {
            atTargetLoopCount = REQUIRED_AT_TARGET_LOOPS;
            return;
        }
        if (Math.abs(lastFinalError) <= positionToleranceTicks) {
            atTargetLoopCount++;
        } else {
            atTargetLoopCount = 0;
        }
    }

    private void updateDisruptionPhase() {
        boolean disruptionEligible = runDisruptionPhase && actuatorMode != ActuatorMode.STANDARD_SERVO_OPEN_LOOP;
        double readyBand = Math.max(positionToleranceTicks, Math.abs(requestedTargetTicks) * disruptionReadyBandPct);
        double detectBand = Math.max(positionToleranceTicks, Math.abs(requestedTargetTicks) * disruptionDropThresholdPct);
        disruptionPhase.update(
            disruptionEligible,
            mode,
            lastFinalError,
            readyBand,
            detectBand,
            disruptionSamples,
            disruptionReadyStableMs,
            disruptionDetectTimeoutMs,
            disruptionRecoveryTimeoutMs
        );
    }

    private void pushTelemetry() {
        String stability = isAtTarget() ? "Stable" : "Adjusting";
        String statusLine = actuatorMode == ActuatorMode.STANDARD_SERVO_OPEN_LOOP
            ? String.format("Mode: %s | Open loop servo | Command: %.2f | %s", mode.name(), lastActuatorCommand, stability)
            : String.format("Mode: %s | Error: %.1f ticks | Output: %.2f | %s", mode.name(), lastFinalError, lastActuatorCommand, stability);
        double gravityTerm = lastGravityFeedforwardTerm + lastCosineFeedforwardTerm;

        if (tunerPhase == TunerPhase.RELAY_TUNING) {
            PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, "[RELAY AUTO-TUNE] Oscillating around target");
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/cyclesCompleted", relayAutoTuner.getCyclesCompleted() + "/" + relayAutoTuner.getCyclesNeeded());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/amplitudeTicks", relayAutoTuner.getOscillationAmplitudeTicks());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/periodSec", relayAutoTuner.getOscillationPeriodSec());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/elapsedSec", String.format(Locale.US, "%.1f / %.1f", relayAutoTuner.getElapsedSec(), relayAutoTuner.getMaxSec()));
            PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, "Do NOT touch the mechanism.");
        } else if (tunerPhase == TunerPhase.RELAY_COMPLETE) {
            GainSet dm = relayComputedMaintainGains;
            GainSet dr = relayComputedRevUpGains;
            PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, "[RELAY AUTO-TUNE] Complete!");
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/Ku", relayAutoTuner.getKu());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/Pu", relayAutoTuner.getPu());
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/computedMaintainKP", dm == null ? 0.0 : dm.kP);
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/computedMaintainKI", dm == null ? 0.0 : dm.kI);
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/computedMaintainKD", dm == null ? 0.0 : dm.kD);
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/computedRevUpKP", dr == null ? 0.0 : dr.kP);
            PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "RelayTune/computedRevUpKD", dr == null ? 0.0 : dr.kD);
            PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry,
                String.format(Locale.US, "PID loop starts in %.1f s...", Math.max(0.0, RELAY_COMPLETE_HOLD_SECONDS - getPhaseElapsedSeconds())));
        }

        PIDFTunerOpMode.addLine(driverTelemetry, dashboardTelemetry, statusLine);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Setpoint/Target", requestedTargetTicks);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Setpoint/RequestedTarget", unclampedRequestedTargetTicks);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Setpoint/ProfiledTarget", profiledTargetTicks);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Setpoint/ProfiledVelocity", profiledVelocityTicksPerSecond);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Setpoint/ProfiledAcceleration", profiledAccelerationTicksPerSecondSquared);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Measurement/Position", averagePositionTicks);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Error/Error", lastFinalError);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Error/ErrorRate", controller.getLastErrorRate());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Error/IntegralSum", controller.getIntegralSum());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/pTerm", controller.getPTerm());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/iTerm", controller.getITerm());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/dTerm", controller.getDTerm());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/fTerm", lastStaticTrimTerm);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/gravityTerm", gravityTerm);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/totalFF", lastFeedforwardTerm);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/fStatic", lastStaticTrimTerm);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/fGravity", lastGravityFeedforwardTerm);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/fCosine", lastCosineFeedforwardTerm);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Terms/output", lastOutput);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Gains/activekF", resolveActivePositionKf());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/loopTimeMs", lastLoopTimeSeconds * MILLIS_PER_SECOND);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/phase", tunerPhase.name());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/relayTuneNote", relayTuneNote.isEmpty() ? "none" : relayTuneNote);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/mode", mode.name());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/isAtTarget", isAtTarget());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/profileEnabled", actuatorMode != ActuatorMode.STANDARD_SERVO_OPEN_LOOP && mode == PIDFTuningMode.REV_UP && motionProfileEnabled);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/actuatorMode", actuatorMode.label);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/feedbackMode", servoFeedbackMode.label);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Diagnostics/actuatorCommand", lastActuatorCommand);
        PIDFTunerOpMode.addData(
            driverTelemetry,
            dashboardTelemetry,
            "Diagnostics/positionConstraintStatus",
            positionConstraintStatus
        );
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Constraints/enabled", positionConstraintsConfigured);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Constraints/minTicks", minPositionTicks);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Constraints/maxTicks", maxPositionTicks);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/state", disruptionPhase.stage.label);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/samples", disruptionPhase.samplesCompleted + "/" + disruptionSamples);
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/meanRecoveryMs", disruptionPhase.meanRecoveryMs());
        PIDFTunerOpMode.addData(driverTelemetry, dashboardTelemetry, "Disruption/worstRecoveryMs", disruptionPhase.worstRecoveryMs);
        PIDFTunerOpMode.addLine(
            driverTelemetry,
            dashboardTelemetry,
            "Guide | kP up=faster, too high=overshoot | kI up=holds load, too high=sluggish recovery"
        );
        PIDFTunerOpMode.addLine(
            driverTelemetry,
            dashboardTelemetry,
            "Guide | kD up=less ringing, too high=noisy | kF/kG/kCos fight friction and gravity"
        );
        PIDFTunerOpMode.updateTelemetry(driverTelemetry, dashboardTelemetry);
    }

    private MotionState calculateMotionState(double totalDistanceTicks, double elapsedSeconds) {
        double direction = Math.signum(totalDistanceTicks);
        double distance = Math.abs(totalDistanceTicks);
        if (distance <= EPSILON) {
            return MotionState.COMPLETE;
        }

        double accelTime = maxProfileVelocity / maxProfileAcceleration;
        double accelDistance = 0.5 * maxProfileAcceleration * accelTime * accelTime;
        boolean triangular = (2.0 * accelDistance) > distance;
        double cruiseTime;
        double peakVelocity;

        if (triangular) {
            accelTime = Math.sqrt(distance / maxProfileAcceleration);
            accelDistance = 0.5 * maxProfileAcceleration * accelTime * accelTime;
            cruiseTime = 0.0;
            peakVelocity = maxProfileAcceleration * accelTime;
        } else {
            peakVelocity = maxProfileVelocity;
            cruiseTime = (distance - (2.0 * accelDistance)) / peakVelocity;
        }

        double decelStart = accelTime + cruiseTime;
        double totalTime = decelStart + accelTime;
        if (elapsedSeconds >= totalTime) {
            return new MotionState(totalDistanceTicks, 0.0, 0.0);
        }
        if (elapsedSeconds < accelTime) {
            return new MotionState(
                direction * 0.5 * maxProfileAcceleration * elapsedSeconds * elapsedSeconds,
                direction * maxProfileAcceleration * elapsedSeconds,
                direction * maxProfileAcceleration
            );
        }
        if (elapsedSeconds < decelStart) {
            double cruiseElapsed = elapsedSeconds - accelTime;
            return new MotionState(
                direction * (accelDistance + (peakVelocity * cruiseElapsed)),
                direction * peakVelocity,
                0.0
            );
        }

        double decelElapsed = elapsedSeconds - decelStart;
        double cruiseDistance = triangular ? 0.0 : peakVelocity * cruiseTime;
        return new MotionState(
            direction * (accelDistance + cruiseDistance + (peakVelocity * decelElapsed)
                - (0.5 * maxProfileAcceleration * decelElapsed * decelElapsed)),
            direction * (peakVelocity - (maxProfileAcceleration * decelElapsed)),
            -direction * maxProfileAcceleration
        );
    }

    private void applyMotorPower(double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    private void applyCRServoPower(double power) {
        for (CRServo servo : crServos) {
            servo.setPower(power);
        }
    }

    private void applyStandardServoPosition(double position) {
        for (Servo servo : standardServos) {
            servo.setPosition(position);
        }
    }

    private static double averageMotorPositions(DcMotorEx[] encoders) {
        double sum = 0.0;
        for (DcMotorEx encoder : encoders) {
            sum += encoder.getCurrentPosition();
        }
        return sum / encoders.length;
    }

    private double mapTicksToServoPosition(double ticks) {
        return clip((ticks - servoRangeMinTicks) / servoRangeTicksPerUnit, MIN_SERVO_POSITION, MAX_SERVO_POSITION);
    }

    private double constrainTargetTicks(double ticks) {
        if (!positionConstraintsConfigured) {
            positionConstraintStatus = "none";
            return ticks;
        }
        double constrained = clip(ticks, minPositionTicks, maxPositionTicks);
        if (Math.abs(constrained - ticks) > EPSILON) {
            positionConstraintStatus = String.format(
                "target clamped from %.1f to %.1f",
                ticks,
                constrained
            );
        } else {
            positionConstraintStatus = "within bounds";
        }
        return constrained;
    }

    private double constrainClosedLoopOutput(double output) {
        if (!positionConstraintsConfigured) {
            return output;
        }
        if (averagePositionTicks >= maxPositionTicks && output > 0.0) {
            positionConstraintStatus = "upper limit reached: outward output blocked";
            return 0.0;
        }
        if (averagePositionTicks <= minPositionTicks && output < 0.0) {
            positionConstraintStatus = "lower limit reached: outward output blocked";
            return 0.0;
        }
        if (Math.abs(unclampedRequestedTargetTicks - requestedTargetTicks) <= EPSILON) {
            positionConstraintStatus = "within bounds";
        }
        return output;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private enum ActuatorMode {
        MOTOR("MOTOR"),
        STANDARD_SERVO_OPEN_LOOP("SERVO_OPEN_LOOP"),
        STANDARD_SERVO_CLOSED_LOOP("SERVO_WITH_EXTERNAL_ENCODER"),
        CR_SERVO("CR_SERVO");

        private final String label;

        ActuatorMode(String label) {
            this.label = label;
        }
    }

    private enum ServoFeedbackMode {
        NONE("NONE"),
        MOTOR_ENCODER("MOTOR_ENCODER"),
        ANALOG_INPUT("ANALOG_INPUT");

        private final String label;

        ServoFeedbackMode(String label) {
            this.label = label;
        }
    }

    private static final class GainSet {
        private final double kP;
        private final double kI;
        private final double kD;
        private final double kF;

        private GainSet(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }

    private static final class MotionState {
        private static final MotionState COMPLETE = new MotionState(0.0, 0.0, 0.0);

        private final double positionTicks;
        private final double velocityTicksPerSecond;
        private final double accelerationTicksPerSecondSquared;

        private MotionState(double positionTicks, double velocityTicksPerSecond, double accelerationTicksPerSecondSquared) {
            this.positionTicks = positionTicks;
            this.velocityTicksPerSecond = velocityTicksPerSecond;
            this.accelerationTicksPerSecondSquared = accelerationTicksPerSecondSquared;
        }
    }

    private static final class DisruptionPhase {
        private Stage stage = Stage.WAITING;
        private long readySinceNs;
        private long detectStartNs;
        private long recoveryStartNs;
        private int samplesCompleted;
        private double totalRecoveryMs;
        private double worstRecoveryMs;
        private boolean anyTimeout;

        private void reset() {
            stage = Stage.WAITING;
            readySinceNs = 0L;
            detectStartNs = 0L;
            recoveryStartNs = 0L;
            samplesCompleted = 0;
            totalRecoveryMs = 0.0;
            worstRecoveryMs = 0.0;
            anyTimeout = false;
        }

        private void update(boolean enabled,
                            PIDFTuningMode mode,
                            double errorTicks,
                            double readyBandTicks,
                            double detectBandTicks,
                            int requiredSamples,
                            long readyStableMs,
                            long detectTimeoutMs,
                            long recoveryTimeoutMs) {
            long nowNs = System.nanoTime();
            if (!enabled || mode != PIDFTuningMode.MAINTAIN) {
                stage = Stage.WAITING;
                readySinceNs = 0L;
                detectStartNs = 0L;
                recoveryStartNs = 0L;
                return;
            }
            if (samplesCompleted >= requiredSamples) {
                stage = Stage.COMPLETE;
                return;
            }

            boolean ready = Math.abs(errorTicks) <= readyBandTicks;
            boolean disturbed = Math.abs(errorTicks) >= detectBandTicks;

            switch (stage) {
                case WAITING:
                    if (ready) {
                        if (readySinceNs == 0L) {
                            readySinceNs = nowNs;
                        }
                        if (elapsedMs(readySinceNs, nowNs) >= readyStableMs) {
                            stage = Stage.ARMED;
                        }
                    } else {
                        readySinceNs = 0L;
                    }
                    break;
                case ARMED:
                    stage = Stage.DETECTING;
                    detectStartNs = nowNs;
                    break;
                case DETECTING:
                    if (disturbed) {
                        stage = Stage.RECOVERING;
                        recoveryStartNs = nowNs;
                    } else if (elapsedMs(detectStartNs, nowNs) >= detectTimeoutMs) {
                        stage = Stage.WAITING;
                        readySinceNs = 0L;
                    }
                    break;
                case RECOVERING:
                    if (ready) {
                        recordRecovery(elapsedMs(recoveryStartNs, nowNs), false);
                    } else if (elapsedMs(recoveryStartNs, nowNs) >= recoveryTimeoutMs) {
                        recordRecovery(recoveryTimeoutMs, true);
                    }
                    break;
                case COMPLETE:
                    break;
            }
        }

        private void recordRecovery(double recoveryMs, boolean timedOut) {
            samplesCompleted++;
            totalRecoveryMs += recoveryMs;
            worstRecoveryMs = Math.max(worstRecoveryMs, recoveryMs);
            anyTimeout |= timedOut;
            stage = Stage.WAITING;
            readySinceNs = 0L;
            detectStartNs = 0L;
            recoveryStartNs = 0L;
        }

        private double meanRecoveryMs() {
            return samplesCompleted == 0 ? 0.0 : totalRecoveryMs / samplesCompleted;
        }

        private String summary(boolean openLoopMode) {
            if (openLoopMode) {
                return "Disruption phase: unavailable in SERVO_OPEN_LOOP because no feedback exists.";
            }
            if (samplesCompleted == 0) {
                return "Disruption phase: no completed recovery samples.";
            }
            return String.format(
                "Disruption phase: mean %.0f ms | worst %.0f ms | timeout=%s",
                meanRecoveryMs(),
                worstRecoveryMs,
                anyTimeout
            );
        }

        private static double elapsedMs(long startNs, long endNs) {
            return (endNs - startNs) / 1e6;
        }
    }

    private enum Stage {
        WAITING("WAITING"),
        ARMED("ARMED: DISRUPT NOW"),
        DETECTING("DETECTING"),
        RECOVERING("RECOVERING"),
        COMPLETE("COMPLETE");

        private final String label;

        Stage(String label) {
            this.label = label;
        }
    }

    enum TunerPhase {
        RELAY_TUNING, RELAY_COMPLETE, RUNNING
    }

    private enum RelayState { OSCILLATING, COMPUTING, COMPLETE }
    private enum CrossingState { ABOVE, BELOW }

    /**
     * Åström–Hägglund relay feedback auto-tune, adapted from VelocityPIDFTuner's for position
     * control. Closing a bang-bang relay on position error drives the mechanism to the target and
     * induces a stable limit cycle once it arrives — no separate "approach target" phase needed.
     * Ku/Pu are measured in raw ticks, then computeGains() scales kP/kI/kD by moveScaleTicks since
     * the PID controller consumes error normalized by that same factor.
     */
    private final class RelayAutoTuner {
        private final double[] recentPositiveIntervalsSec = new double[RELAY_REQUIRED_CYCLES];
        private final double[] recentHalfCyclePeaks = new double[RELAY_REQUIRED_CYCLES];
        private RelayState state = RelayState.OSCILLATING;
        private CrossingState crossingState = CrossingState.ABOVE;
        private long stateStartNs;
        private long lastPositiveCrossingNs;
        private int positiveIntervalCount, positiveIntervalIndex, halfCyclePeakCount, halfCyclePeakIndex, cyclesCompleted;
        private double relayOutput, currentHalfCyclePeak, oscillationAmplitudeTicks, oscillationPeriodSec, ku, pu;
        private GainSet computedMaintainGains, computedRevUpGains;
        private boolean sawAboveBandSinceCrossing, sawBelowBandSinceCrossing;
        private boolean completedSuccessfully, skipToRunning;
        private String completionMessage = "";

        private void reset() {
            state = RelayState.OSCILLATING;
            crossingState = CrossingState.ABOVE;
            stateStartNs = System.nanoTime();
            lastPositiveCrossingNs = 0L;
            positiveIntervalCount = 0;
            positiveIntervalIndex = 0;
            halfCyclePeakCount = 0;
            halfCyclePeakIndex = 0;
            cyclesCompleted = 0;
            relayOutput = 0.0;
            currentHalfCyclePeak = 0.0;
            oscillationAmplitudeTicks = 0.0;
            oscillationPeriodSec = 0.0;
            ku = 0.0;
            pu = 0.0;
            computedMaintainGains = null;
            computedRevUpGains = null;
            sawAboveBandSinceCrossing = false;
            sawBelowBandSinceCrossing = false;
            completedSuccessfully = false;
            skipToRunning = false;
            completionMessage = "";
        }

        private double update(double error) {
            long nowNs = System.nanoTime();
            double hysteresisBand = getHysteresisBandTicks();

            if (state == RelayState.OSCILLATING) {
                trackOscillation(error, hysteresisBand, nowNs);
                if (error > hysteresisBand) relayOutput = relayAmplitude;
                else if (error < -hysteresisBand) relayOutput = -relayAmplitude;
                if (cyclesCompleted >= RELAY_REQUIRED_CYCLES
                        && oscillationAmplitudeTicks > EPSILON && oscillationPeriodSec > EPSILON) {
                    state = RelayState.COMPUTING;
                } else if (elapsedSeconds(stateStartNs, nowNs) >= RELAY_MAX_OSCILLATION_SECONDS) {
                    relayOutput = 0.0;
                    skipToRunning = true;
                    completionMessage = "Relay oscillation timeout: try increasing relayAmplitude, or check for "
                        + "excess friction/backlash. No gains were changed — tune manually instead.";
                    return relayOutput;
                }
            }

            if (state == RelayState.COMPUTING) {
                computeGains();
                relayOutput = 0.0;
                state = RelayState.COMPLETE;
                completedSuccessfully = true;
                completionMessage = "Relay auto-tune complete.";
            }
            return relayOutput;
        }

        private void trackOscillation(double error, double hysteresisBand, long nowNs) {
            currentHalfCyclePeak = Math.max(currentHalfCyclePeak, Math.abs(error));
            if (error > hysteresisBand) sawAboveBandSinceCrossing = true;
            else if (error < -hysteresisBand) sawBelowBandSinceCrossing = true;
            if (crossingState == CrossingState.ABOVE && sawAboveBandSinceCrossing && error < -hysteresisBand) {
                recordHalfCyclePeak();
                crossingState = CrossingState.BELOW;
                sawAboveBandSinceCrossing = false;
                sawBelowBandSinceCrossing = true;
                currentHalfCyclePeak = Math.abs(error);
            } else if (crossingState == CrossingState.BELOW && sawBelowBandSinceCrossing && error > hysteresisBand) {
                recordHalfCyclePeak();
                recordPositiveCrossing(nowNs);
                crossingState = CrossingState.ABOVE;
                sawBelowBandSinceCrossing = false;
                sawAboveBandSinceCrossing = true;
                currentHalfCyclePeak = Math.abs(error);
            }
        }

        private void recordHalfCyclePeak() {
            recentHalfCyclePeaks[halfCyclePeakIndex] = currentHalfCyclePeak;
            halfCyclePeakIndex = (halfCyclePeakIndex + 1) % recentHalfCyclePeaks.length;
            halfCyclePeakCount = Math.min(halfCyclePeakCount + 1, recentHalfCyclePeaks.length);
            oscillationAmplitudeTicks = average(recentHalfCyclePeaks, halfCyclePeakCount);
        }

        private void recordPositiveCrossing(long nowNs) {
            if (lastPositiveCrossingNs != 0L) {
                recentPositiveIntervalsSec[positiveIntervalIndex] = elapsedSeconds(lastPositiveCrossingNs, nowNs);
                positiveIntervalIndex = (positiveIntervalIndex + 1) % recentPositiveIntervalsSec.length;
                positiveIntervalCount = Math.min(positiveIntervalCount + 1, recentPositiveIntervalsSec.length);
                cyclesCompleted++;
                oscillationPeriodSec = average(recentPositiveIntervalsSec, positiveIntervalCount);
                pu = oscillationPeriodSec;
            }
            lastPositiveCrossingNs = nowNs;
        }

        private void computeGains() {
            double amplitude = Math.max(oscillationAmplitudeTicks, EPSILON);
            ku = (4.0 * relayAmplitude) / (Math.PI * amplitude);
            pu = oscillationPeriodSec;
            // Ku/Pu describe the plant in raw ticks; the PID controller consumes error normalized
            // by moveScaleTicks, so kP/kI/kD must scale up by the same factor for equivalent effect.
            // moveScaleTicks is NOT maintained during relay tuning (updateProfile is skipped), so use
            // the value the RUNNING phase will actually normalize by, which the handoff also locks in.
            double scale = resolveHoldMoveScaleTicks();
            double bMKp = 0.3 * ku * scale, bMKi = bMKp / Math.max(pu, EPSILON), bMKd = (bMKp * pu) / 8.0;
            double bRKp = 0.5 * ku * scale, bRKd = (bRKp * pu) / 20.0;
            computedMaintainGains = new GainSet(relayDetune * bMKp, relayDetune * bMKi, relayDetune * bMKd, 0.0);
            computedRevUpGains = new GainSet(relayDetune * bRKp, 0.0, relayDetune * bRKd, 0.0);
        }

        GainSet getComputedMaintainGains() { return computedMaintainGains; }
        GainSet getComputedRevUpGains() { return computedRevUpGains; }
        String getStateName() { return state.name(); }
        int getCyclesCompleted() { return Math.min(cyclesCompleted, RELAY_REQUIRED_CYCLES); }
        int getCyclesNeeded() { return RELAY_REQUIRED_CYCLES; }
        double getOscillationAmplitudeTicks() { return oscillationAmplitudeTicks; }
        double getOscillationPeriodSec() { return oscillationPeriodSec; }
        double getKu() { return ku; }
        double getPu() { return pu; }
        double getElapsedSec() { return elapsedSeconds(stateStartNs, System.nanoTime()); }
        double getMaxSec() { return RELAY_MAX_OSCILLATION_SECONDS; }
        double getHysteresisBandTicks() {
            return Math.max(positionToleranceTicks, Math.abs(requestedTargetTicks) * relayHysteresisBandPct);
        }
        boolean hasCompletedSuccessfully() { return completedSuccessfully; }
        boolean shouldSkipToRunning() { return skipToRunning; }
        String getCompletionMessage() { return completionMessage; }

        private double average(double[] values, int count) {
            if (count <= 0) return 0.0;
            double sum = 0.0;
            for (int i = 0; i < count; i++) sum += values[i];
            return sum / count;
        }

        private double elapsedSeconds(long startNs, long endNs) {
            return (endNs - startNs) * 1e-9;
        }
    }

    public static class Config {
        private Double targetTicks;
        private PIDFTuningMode tuningMode = PIDFTuningMode.MAINTAIN;
        private DcMotorEx[] motors;
        private Servo[] standardServos;
        private CRServo[] crServos;
        private DcMotorEx[] crServoFeedbackEncoders;
        private DcMotorEx servoFeedbackEncoderMotor;
        private AnalogInput servoFeedbackAnalogInput;
        private Telemetry telemetry;
        private GainSet revUpGains;
        private GainSet maintainGains;
        private Double integralSumMax;
        private Double derivativeAlpha;
        private boolean motionProfileEnabled;
        private double maxProfileVelocity;
        private double maxProfileAcceleration;
        private double positionToleranceTicks = 10.0;
        private double feedforwardGravityConstant;
        private double feedforwardCosineConstant;
        private double cosineZeroTicks;
        private double cosineTicksPerRadian;
        private boolean cosineReferenceConfigured;
        private boolean servoRangeConfigured;
        private double servoRangeMinTicks;
        private double servoRangeMaxTicks;
        private double analogVoltageToTicksScale;
        private double servoOutputScale = 1.0;
        private boolean positionConstraintsConfigured;
        private double minPositionTicks;
        private double maxPositionTicks;
        private boolean runDisruptionPhase;
        private int disruptionSamples = 3;
        private long disruptionReadyStableMs = 500;
        private long disruptionDetectTimeoutMs = 5000;
        private long disruptionRecoveryTimeoutMs = 3000;
        private double disruptionReadyBandPct = 0.05;
        private double disruptionDropThresholdPct = 0.08;
        private boolean skipRelayTuning;
        private double relayAmplitude = DEFAULT_RELAY_AMPLITUDE;
        private double relayHysteresisBandPct = DEFAULT_RELAY_HYSTERESIS_BAND_PCT;
        private double relayDetune = DEFAULT_RELAY_DETUNE;

        /**
         * Sets the position target in ticks or any other units shared by the feedback source. Default:
         * required. Raise or lower this when you want to tune a different arm, elevator, slide, or servo
         * position in the same units as your encoder or analog conversion.
         */
        public Config target(double ticks) {
            targetTicks = ticks;
            return this;
        }

        /**
         * Chooses the active behavior mode. Default: {@link PIDFTuningMode#MAINTAIN}. Use
         * {@code REV_UP} for fast moves and {@code MAINTAIN} for holding load at the destination.
         */
        public Config tuningMode(PIDFTuningMode mode) {
            tuningMode = mode == null ? PIDFTuningMode.MAINTAIN : mode;
            return this;
        }

        /**
         * Supplies one or more DC motors driven by the same position command. Default: required unless
         * you are using servos. Add all linked motors here when the mechanism twists because only one
         * motor was included.
         */
        public Config withMotors(DcMotorEx... motors) {
            this.motors = motors;
            return this;
        }

        /**
         * Supplies one or more standard positional servos. Default: not used. If you also add
         * {@link #withServoFeedback(DcMotorEx)} or {@link #withServoFeedbackAnalog(AnalogInput, double)},
         * this enters {@code SERVO_WITH_EXTERNAL_ENCODER}. Without feedback, it enters
         * {@code SERVO_OPEN_LOOP}. For closed-loop servo control, also add
         * {@link #withServoOpenLoopRange(double, double)} so target units can be mapped back to
         * {@code Servo.setPosition()}.
         */
        public Config withServos(Servo... servos) {
            standardServos = servos;
            return this;
        }

        /**
         * Supplies an external encoder motor for standard servo closed-loop feedback. Default: not used.
         * Pass the {@code DcMotorEx} that shares a shaft with your servo. Set its mode to
         * {@code RUN_WITHOUT_ENCODER}. Never call {@code setPower()} on it.
         */
        public Config withServoFeedback(DcMotorEx encoderMotor) {
            servoFeedbackEncoderMotor = encoderMotor;
            return this;
        }

        /**
         * Supplies analog feedback for standard servo closed-loop control. Default: not used. The tuner
         * reads {@code analogInput.getVoltage()} and multiplies by {@code voltageToTicksScale}. Calibration
         * tip: command the servo to 0.0 and 1.0, read the analog voltage at each extreme, then estimate
         * {@code scale = maxTicks / (voltageAtMax - voltageAtMin)} for the controlled travel range.
         */
        public Config withServoFeedbackAnalog(AnalogInput analogInput, double voltageToTicksScale) {
            servoFeedbackAnalogInput = analogInput;
            analogVoltageToTicksScale = voltageToTicksScale;
            return this;
        }

        /**
         * Defines the target range that maps linearly to {@code Servo.setPosition(0.0..1.0)}. Default:
         * required for {@code SERVO_OPEN_LOOP}, and also required for closed-loop standard servos because
         * the tuner must map target units back to servo position commands.
         */
        public Config withServoOpenLoopRange(double minTicks, double maxTicks) {
            servoRangeConfigured = true;
            servoRangeMinTicks = minTicks;
            servoRangeMaxTicks = maxTicks;
            return this;
        }

        /**
         * Supplies one CR servo and its feedback encoder(s). Default: not used. Power is applied with
         * {@code CRServo.setPower()} and the loop behaves like motor position control. Provide either one
         * shared encoder or one encoder per servo.
         */
        public Config withCRServos(CRServo servo, DcMotorEx... feedbackEncoders) {
            return withCRServos(new CRServo[]{servo}, feedbackEncoders);
        }

        /**
         * Supplies multiple CR servos and their feedback encoder(s). Java cannot express two varargs in
         * one method signature, so the servos are passed as an array and the encoders remain varargs.
         * Provide either one shared encoder or one encoder per servo.
         */
        public Config withCRServos(CRServo[] servos, DcMotorEx... feedbackEncoders) {
            crServos = servos;
            crServoFeedbackEncoders = feedbackEncoders;
            return this;
        }

        /**
         * Scales the final actuator command before calling {@code setPosition()} or {@code setPower()}.
         * Default: {@code 1.0}. Lower it if the mechanism is mechanically geared and full output is too
         * aggressive. Standard servo commands are clamped to {@code [0.0, 1.0]}; CR servo commands are
         * clamped to {@code [-1.0, 1.0]}.
         */
        public Config servoOutputScale(double scale) {
            servoOutputScale = scale;
            return this;
        }

        /**
         * Overrides the REV_UP PIDF gains. Default: Dashboard fields
         * ({@code 0.9, 0.0, 0.12, 0.0}). {@code kF} is static trim in direct actuator-power units, so
         * raise it only when the mechanism needs extra push to break friction at the start of a move.
         */
        public Config revUpGains(double kP, double kI, double kD, double kF) {
            revUpGains = new GainSet(kP, kI, kD, kF);
            return this;
        }

        /**
         * Overrides the MAINTAIN PIDF gains. Default: Dashboard fields
         * ({@code 0.6, 0.03, 0.08, 0.0}). {@code kF} is static trim in direct actuator-power units,
         * while gravity and arm-angle compensation belong in {@code kG}/{@code kCos}.
         */
        public Config maintainGains(double kP, double kI, double kD, double kF) {
            maintainGains = new GainSet(kP, kI, kD, kF);
            return this;
        }

        /**
         * Disables relay auto-tune. Default: enabled for MOTOR and CR_SERVO actuators unless both
         * {@link #revUpGains} and {@link #maintainGains} are already supplied. Call this if you want
         * to run cold with the Dashboard default gains instead of auto-tuning on every start.
         */
        public Config skipRelayTuning() {
            skipRelayTuning = true;
            return this;
        }

        /**
         * Sets the bang-bang power magnitude used while measuring the relay auto-tune oscillation.
         * Default: {@code 0.3}. Raise it if the mechanism cannot overcome friction/gravity enough to
         * oscillate; lower it if the oscillation is violent enough to risk hardware.
         */
        public Config relayAmplitude(double amplitude) {
            relayAmplitude = clip(amplitude, MIN_RELAY_AMPLITUDE, MAX_RELAY_AMPLITUDE);
            return this;
        }

        /**
         * Sets the relay switching band as a fraction of {@code |target|}. Default: {@code 0.03}.
         * Raise it if position noise causes chattering false crossings.
         */
        public Config relayHysteresisBandPct(double pct) {
            relayHysteresisBandPct = Math.max(0.0, pct);
            return this;
        }

        /**
         * Scales the relay-computed gains after auto-tune. Default: {@code 1.0}. Lower it for a more
         * conservative start; raise it (up to {@code 2.0}) for a more aggressive one.
         */
        public Config relayDetune(double factor) {
            relayDetune = clip(factor, MIN_RELAY_DETUNE, MAX_RELAY_DETUNE);
            return this;
        }

        /**
         * Caps the integral contribution in actuator-output units. Default: {@code 0.25}. Lower it if a
         * held mechanism overshoots after being displaced for a long time.
         */
        public Config integralSumMax(double max) {
            integralSumMax = max;
            return this;
        }

        /**
         * Sets derivative low-pass alpha. Default: {@code 0.2}. Lower it when encoder noise makes
         * derivative chatter; raise it when damping feels delayed.
         */
        public Config derivativeAlpha(double alpha) {
            derivativeAlpha = alpha;
            return this;
        }

        /**
         * Enables a trapezoidal profile for REV_UP moves. Default: disabled. Turn this on when a raw
         * step command slams the mechanism or makes large moves rough and saturated. This does not apply
         * in {@code SERVO_OPEN_LOOP}, which intentionally stays a direct position mapper.
         */
        public Config useMotionProfile(double maxVelocity, double maxAcceleration) {
            motionProfileEnabled = true;
            maxProfileVelocity = maxVelocity;
            maxProfileAcceleration = maxAcceleration;
            return this;
        }

        /**
         * Sets the final position tolerance in ticks or feedback units. Default: {@code 10}. Raise it if
         * the encoder is coarse or noisy; lower it for tighter settling requirements.
         */
        public Config positionToleranceTicks(double ticks) {
            positionToleranceTicks = ticks;
            return this;
        }

        /**
         * Adds optional hard position bounds in the same units as the feedback source. Targets are
         * clamped into this window, and any output that would keep driving farther outward once the
         * mechanism is already at a limit is suppressed. Use this to protect arms, slides, turrets, or
         * other mechanisms with fragile end stops.
         */
        public Config positionBounds(double minTicks, double maxTicks) {
            positionConstraintsConfigured = true;
            minPositionTicks = minTicks;
            maxPositionTicks = maxTicks;
            return this;
        }

        /**
         * Adds constant upward feedforward for vertical mechanisms. Default: {@code 0.0}. Increase it
         * when an elevator or CR-servo mechanism droops under gravity before PID can react.
         */
        public Config feedforwardGravityConstant(double kG) {
            feedforwardGravityConstant = kG;
            return this;
        }

        /**
         * Adds cosine gravity feedforward for arms. Default: {@code 0.0}. Increase it when an arm needs
         * extra hold torque near horizontal and less near vertical.
         */
        public Config feedforwardCosineConstant(double kCos) {
            feedforwardCosineConstant = kCos;
            return this;
        }

        /**
         * Defines the encoder-to-angle conversion used by cosine feedforward. Default: required only if
         * {@code kCos != 0}. Adjust this when cosine compensation peaks at the wrong arm angle.
         */
        public Config cosineFeedforwardReference(double zeroTicks, double ticksPerRadian) {
            cosineZeroTicks = zeroTicks;
            cosineTicksPerRadian = ticksPerRadian;
            cosineReferenceConfigured = true;
            return this;
        }

        /**
         * Enables the hold robustness test after the mechanism stabilizes. Default: {@code false}. Turn
         * this on when you want measured disturbance-recovery times instead of only a step response.
         * This is ignored in {@code SERVO_OPEN_LOOP} because no feedback exists.
         */
        public Config runDisruptionPhase(boolean run) {
            runDisruptionPhase = run;
            return this;
        }

        /**
         * Sets how many successful recovery samples the disruption phase records. Default: {@code 3}.
         * Raise it when real disturbances vary a lot between pushes.
         */
        public Config disruptionSamples(int n) {
            disruptionSamples = n;
            return this;
        }

        /**
         * Sets how long the mechanism must stay in-band before arming a disruption. Default:
         * {@code 500 ms}. Raise it if the test arms while the mechanism is still bouncing.
         */
        public Config disruptionReadyStableMs(long ms) {
            disruptionReadyStableMs = ms;
            return this;
        }

        /**
         * Sets how long the tuner waits for the user to disturb the mechanism. Default: {@code 5000 ms}.
         * Raise it if the operator needs more time to push or load the mechanism.
         */
        public Config disruptionDetectTimeoutMs(long ms) {
            disruptionDetectTimeoutMs = ms;
            return this;
        }

        /**
         * Sets the maximum allowed recovery time for one disturbance sample. Default: {@code 3000 ms}.
         * Lower it when you want a stricter hold-performance requirement.
         */
        public Config disruptionRecoveryTimeoutMs(long ms) {
            disruptionRecoveryTimeoutMs = ms;
            return this;
        }

        /**
         * Sets the ready band as a fraction of target position magnitude. Default: {@code 0.05}. Raise
         * it if the disruption test never arms; lower it if you want a tighter steady-state definition.
         */
        public Config disruptionReadyBandPct(double pct) {
            disruptionReadyBandPct = pct;
            return this;
        }

        /**
         * Sets the disturbance detection band as a fraction of target position magnitude. Default:
         * {@code 0.08}. Raise it if tiny bumps trigger false detections; lower it if real pushes are missed.
         */
        public Config disruptionDropThresholdPct(double pct) {
            disruptionDropThresholdPct = pct;
            return this;
        }

        /**
         * Supplies the OpMode telemetry used for Driver Station output and Dashboard mirroring. Default:
         * required. Add this when the tuner throws a missing-telemetry startup error.
         */
        public Config telemetry(Telemetry telemetry) {
            this.telemetry = telemetry;
            return this;
        }

        PIDFTuningMode getResolvedMode() {
            return tuningMode == null ? PIDFTuningMode.MAINTAIN : tuningMode;
        }

        boolean hasManualRevUpGains() {
            return revUpGains != null;
        }

        boolean hasManualMaintainGains() {
            return maintainGains != null;
        }

        private ActuatorMode resolveActuatorMode() {
            boolean hasMotors = motors != null && motors.length > 0;
            boolean hasStandardServos = standardServos != null && standardServos.length > 0;
            boolean hasCRServos = crServos != null && crServos.length > 0;
            if (hasMotors) {
                return ActuatorMode.MOTOR;
            }
            if (hasCRServos) {
                return ActuatorMode.CR_SERVO;
            }
            if (hasStandardServos) {
                return resolveServoFeedbackMode() == ServoFeedbackMode.NONE
                    ? ActuatorMode.STANDARD_SERVO_OPEN_LOOP
                    : ActuatorMode.STANDARD_SERVO_CLOSED_LOOP;
            }
            return null;
        }

        private ServoFeedbackMode resolveServoFeedbackMode() {
            if (servoFeedbackEncoderMotor != null) {
                return ServoFeedbackMode.MOTOR_ENCODER;
            }
            if (servoFeedbackAnalogInput != null) {
                return ServoFeedbackMode.ANALOG_INPUT;
            }
            return ServoFeedbackMode.NONE;
        }

        private DcMotorEx[] resolveFeedbackEncoders() {
            ActuatorMode actuatorMode = resolveActuatorMode();
            if (actuatorMode == ActuatorMode.STANDARD_SERVO_CLOSED_LOOP && servoFeedbackEncoderMotor != null) {
                return new DcMotorEx[]{servoFeedbackEncoderMotor};
            }
            return crServoFeedbackEncoders;
        }

        private GainSet resolveRevUpGains() {
            return revUpGains == null ? new GainSet(REV_UP_KP, REV_UP_KI, REV_UP_KD, REV_UP_KF) : revUpGains;
        }

        private GainSet resolveMaintainGains() {
            return maintainGains == null ? new GainSet(MAINTAIN_KP, MAINTAIN_KI, MAINTAIN_KD, MAINTAIN_KF) : maintainGains;
        }

        private double resolveIntegralSumMax() {
            return integralSumMax == null ? DEFAULT_INTEGRAL_SUM_MAX : integralSumMax;
        }

        private double resolveDerivativeAlpha() {
            return derivativeAlpha == null ? DEFAULT_DERIVATIVE_ALPHA : derivativeAlpha;
        }

        private void validate() {
            boolean hasMotors = motors != null && motors.length > 0;
            boolean hasStandardServos = standardServos != null && standardServos.length > 0;
            boolean hasCRServos = crServos != null && crServos.length > 0;
            int actuatorFamilies = (hasMotors ? 1 : 0) + (hasStandardServos ? 1 : 0) + (hasCRServos ? 1 : 0);

            if (targetTicks == null) {
                throw new IllegalStateException(
                    "Position tuner missing target. Add .target(TARGET_POSITION) to the returned PositionPIDFTuner.Config.");
            }
            if (actuatorFamilies == 0) {
                throw new IllegalStateException(
                    "Position tuner missing actuator. Add .withMotors(...), .withServos(...), or .withCRServos(...).");
            }
            if (actuatorFamilies > 1) {
                throw new IllegalStateException(
                    "Cannot mix motors and servos in the same PositionPIDFTuner. Create two separate tuners.");
            }
            if (telemetry == null) {
                throw new IllegalStateException(
                    "Position tuner missing telemetry. Add .telemetry(telemetry) to the returned PositionPIDFTuner.Config.");
            }
            if (motionProfileEnabled && (maxProfileVelocity <= 0.0 || maxProfileAcceleration <= 0.0)) {
                throw new IllegalStateException(
                    "Position tuner motion profile needs positive limits. Use .useMotionProfile(maxVelocity, maxAcceleration).");
            }
            if (positionConstraintsConfigured && maxPositionTicks <= minPositionTicks) {
                throw new IllegalStateException(
                    "Position tuner position bounds must have max > min. Use .positionBounds(minTicks, maxTicks).");
            }
            if (servoOutputScale < 0.0) {
                throw new IllegalStateException(
                    "Position tuner servoOutputScale must be non-negative. Use .servoOutputScale(scale) with scale >= 0.");
            }
            if (feedforwardCosineConstant != 0.0 && !cosineReferenceConfigured) {
                throw new IllegalStateException(
                    "Position tuner cosine feedforward needs angle conversion. Add .cosineFeedforwardReference(zeroTicks, ticksPerRadian).");
            }

            if (hasStandardServos) {
                boolean hasEncoderFeedback = servoFeedbackEncoderMotor != null;
                boolean hasAnalogFeedback = servoFeedbackAnalogInput != null;
                if (hasEncoderFeedback && hasAnalogFeedback) {
                    throw new IllegalStateException(
                        "Standard servo control can use only one feedback source. Choose either .withServoFeedback(...) or .withServoFeedbackAnalog(...).");
                }
                if (!servoRangeConfigured || Math.abs(servoRangeMaxTicks - servoRangeMinTicks) <= EPSILON) {
                    throw new IllegalStateException(
                        "Standard servo control needs a target mapping range. Add .withServoOpenLoopRange(minTicks, maxTicks).");
                }
                if (hasAnalogFeedback && analogVoltageToTicksScale == 0.0) {
                    throw new IllegalStateException(
                        "Analog servo feedback needs a non-zero conversion scale. Add .withServoFeedbackAnalog(analogInput, voltageToTicksScale).");
                }
            }

            if (hasCRServos) {
                if (crServoFeedbackEncoders == null || crServoFeedbackEncoders.length == 0) {
                    throw new IllegalStateException(
                        "CR servo control needs feedback encoders. Add .withCRServos(servo, encoder) or the array overload with one shared encoder or one encoder per servo.");
                }
                if (!(crServoFeedbackEncoders.length == 1 || crServoFeedbackEncoders.length == crServos.length)) {
                    throw new IllegalStateException(
                        "CR servo control requires either one shared encoder or one encoder per servo.");
                }
            }
        }
    }
}
