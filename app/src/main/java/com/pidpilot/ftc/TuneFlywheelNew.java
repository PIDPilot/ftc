package com.pidpilot.ftc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Tune Flywheel New", group = "Tuning")
public class TuneFlywheelNew extends PIDFTunerOpMode {
    public static double TARGET_VELOCITY = 640.0;
    public static PIDFTuningMode TUNING_MODE = PIDFTuningMode.MAINTAIN;

    private DcMotorEx intake;
    private DcMotorEx transfer;
    private DcMotorEx left;
    private DcMotorEx right;
    private boolean feederStarted;

    @Override
    protected VelocityPIDFTuner.Config configureVelocity() {
        if (left == null) {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            transfer = hardwareMap.get(DcMotorEx.class, "transfer");
            left = hardwareMap.get(DcMotorEx.class, "outtakeL");
            right = hardwareMap.get(DcMotorEx.class, "outtakeR");
            left.setDirection(DcMotorSimple.Direction.REVERSE);
            right.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        // Only spin the feeder once the match/test has actually started — never during INIT.
        if (!feederStarted && isStarted()) {
            intake.setPower(1);
            transfer.setPower(1);
            feederStarted = true;
        }
        return new VelocityPIDFTuner.Config()
            .target(TARGET_VELOCITY)
            .tuningMode(TUNING_MODE)
            .withMotors(left, right)
            .averageAbsoluteVelocity(true)
            .runDisruptionPhase(true)
            .disruptionSamples(5)
            .telemetry(telemetry);
    }
}
