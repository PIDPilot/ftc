package com.pidpilot.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BiConsumer;
import java.util.function.DoubleConsumer;
import java.util.function.Function;
import java.util.function.Supplier;

public abstract class PIDFTunerOpMode extends LinearOpMode {
    /** Default loop period used on the very first cycle before real timing exists. */
    static final double DEFAULT_LOOP_TIME_SECONDS = 0.02;
    /** Smallest accepted loop dt so the first cycle and very fast loops stay finite. */
    static final double MIN_LOOP_TIME_SECONDS = 0.001;
    /** Largest accepted loop dt so a stalled cycle cannot distort PID math. */
    static final double MAX_LOOP_TIME_SECONDS = 0.1;

    private static Telemetry cachedDashboardTelemetry;
    private static boolean dashboardLookupAttempted;

    protected VelocityPIDFTuner.Config configureVelocity() { return null; }

    protected PositionPIDFTuner.Config configurePosition() { return null; }

    @Override
    public final void runOpMode() {
        VelocityPIDFTuner.Config velocityConfig = configureVelocity();
        PositionPIDFTuner.Config positionConfig = configurePosition();

        if (velocityConfig == null && positionConfig == null) {
            throw new IllegalStateException(
                "No tuner configured. Override configureVelocity() or configurePosition() and return a Config.");
        }
        if (velocityConfig != null && positionConfig != null) {
            throw new IllegalStateException(
                "Only one tuner may be configured. Return a Config from either configureVelocity() or configurePosition(), not both.");
        }

        if (velocityConfig != null) {
            runVelocityTuner(velocityConfig);
            return;
        }
        runPositionTuner(positionConfig);
    }

    static Telemetry getDashboardTelemetry() {
        if (dashboardLookupAttempted) {
            return cachedDashboardTelemetry;
        }
        dashboardLookupAttempted = true;
        try {
            Class<?> dashboardClass = Class.forName("com.acmerobotics.dashboard.FtcDashboard");
            Object dashboard = dashboardClass.getMethod("getInstance").invoke(null);
            Object telemetry = dashboardClass.getMethod("getTelemetry").invoke(dashboard);
            if (telemetry instanceof Telemetry) {
                cachedDashboardTelemetry = (Telemetry) telemetry;
            }
        } catch (Throwable ignored) {
            cachedDashboardTelemetry = null;
        }
        return cachedDashboardTelemetry;
    }

    static void addLine(Telemetry primary, Telemetry secondary, String line) {
        primary.addLine(line);
        if (secondary != null) {
            secondary.addLine(line);
        }
    }

    static void addData(Telemetry primary, Telemetry secondary, String caption, Object value) {
        primary.addData(caption, value);
        if (secondary != null) {
            secondary.addData(caption, value);
        }
    }

    static void updateTelemetry(Telemetry primary, Telemetry secondary) {
        primary.update();
        if (secondary != null) {
            secondary.update();
        }
    }

    private void runVelocityTuner(VelocityPIDFTuner.Config initialConfig) {
        VelocityPIDFTuner tuner = new VelocityPIDFTuner(initialConfig);
        runTunerLoop(
            "Velocity",
            initialConfig.getResolvedMode(),
            this::requireVelocityConfig,
            VelocityPIDFTuner.Config::getResolvedMode,
            tuner::refreshFrom,
            tuner::update,
            tuner::onStart,
            tuner::pushFinalSummary
        );
    }

    private void runPositionTuner(PositionPIDFTuner.Config initialConfig) {
        PositionPIDFTuner tuner = new PositionPIDFTuner(initialConfig);
        runTunerLoop(
            "Position",
            initialConfig.getResolvedMode(),
            this::requirePositionConfig,
            PositionPIDFTuner.Config::getResolvedMode,
            tuner::refreshFrom,
            tuner::update,
            tuner::onStart,
            tuner::pushFinalSummary
        );
    }

    /**
     * Shared drive loop for both tuner types: shows the ready screen, waits for start, then every
     * cycle re-reads the live Dashboard-backed config, applies the X-button REV_UP/MAINTAIN toggle,
     * measures loop dt, and steps the tuner. Velocity and position tuners only differ in their
     * Config type and which tuner instance methods get called, so those are passed in as references.
     */
    private <C> void runTunerLoop(
            String tunerLabel,
            PIDFTuningMode initialMode,
            Supplier<C> requireLiveConfig,
            Function<C, PIDFTuningMode> resolveMode,
            BiConsumer<C, PIDFTuningMode> refreshFrom,
            DoubleConsumer update,
            Runnable onStart,
            Runnable pushFinalSummary) {
        PIDFTuningMode activeMode = initialMode;
        PIDFTuningMode configMode = activeMode;
        boolean lastX = false;

        telemetry.addLine(tunerLabel + " PIDF tuner ready");
        telemetry.addLine("X toggles REV_UP / MAINTAIN");
        telemetry.addData("Start mode", activeMode.name());
        telemetry.update();
        waitForStart();
        if (isStopRequested()) {
            return;
        }
        // The tuner was constructed during INIT, before this point — re-baseline any phase timer
        // that started then, so INIT dwell (routinely tens of seconds in competition) doesn't
        // silently eat into the characterization/relay-tuning time budget.
        onStart.run();

        long previousTimeNs = System.nanoTime();
        while (opModeIsActive() && !isStopRequested()) {
            C liveConfig = requireLiveConfig.get();
            PIDFTuningMode liveMode = resolveMode.apply(liveConfig);
            if (liveMode != configMode) {
                configMode = liveMode;
                activeMode = configMode;
            }
            boolean xPressed = gamepad1.x;
            if (xPressed && !lastX) {
                activeMode = activeMode == PIDFTuningMode.REV_UP ? PIDFTuningMode.MAINTAIN : PIDFTuningMode.REV_UP;
            }
            lastX = xPressed;

            long nowNs = System.nanoTime();
            double loopTimeSeconds = clampLoopTime((nowNs - previousTimeNs) * 1e-9);
            previousTimeNs = nowNs;
            refreshFrom.accept(liveConfig, activeMode);
            update.accept(loopTimeSeconds);
        }
        pushFinalSummary.run();
    }

    private VelocityPIDFTuner.Config requireVelocityConfig() {
        VelocityPIDFTuner.Config config = configureVelocity();
        if (config == null) {
            throw new IllegalStateException(
                "configureVelocity() returned null after the OpMode started. Keep returning the same VelocityPIDFTuner.Config while running.");
        }
        return config;
    }

    private PositionPIDFTuner.Config requirePositionConfig() {
        PositionPIDFTuner.Config config = configurePosition();
        if (config == null) {
            throw new IllegalStateException(
                "configurePosition() returned null after the OpMode started. Keep returning the same PositionPIDFTuner.Config while running.");
        }
        return config;
    }

    private static double clampLoopTime(double loopTimeSeconds) {
        return Math.max(MIN_LOOP_TIME_SECONDS, Math.min(MAX_LOOP_TIME_SECONDS, loopTimeSeconds));
    }
}
