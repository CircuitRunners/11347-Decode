package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Start with kF auto, then raise/lower kF until the motor reaches near target at steady state (low error).
 * Add small kP to tighten response (too high = oscillation).
 * Only add kI if you see steady-state error that kF+kP can’t fix, prob won't be needed
 * Add kD to damp oscillations during spin-up, might not be needed
 */
//@Disabled
@Config
@TeleOp
public class ShooterPIDTest extends CommandOpMode {
    private DcMotorEx shooter1, shooter2;
    public GamepadEx driver;

    // Dashboard-tunable constants
    public static double TARGET_RPM = 3500.0; // 4000
    public static double MOTOR_RPM = 6000.0; // 1410
    public static double GEAR_RATIO = 40.0 / 52.0;
    public static double TICKS_PER_REV = 28;


    // PIDF (velocity)
    public static double kP = 0;
    public static double kI = 0.0;
    public static double kD = 0;
    public static double kF = 0;

    public double kfValue;

    // Toggles shooter on and off in dashboard
    public static boolean runShooter = false;
    private FtcDashboard dash;

    @Override
    public void initialize() {
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        shooter1 = hardwareMap.get(DcMotorEx.class, "m1");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        shooter2 = hardwareMap.get(DcMotorEx.class, "m2");

        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyPIDF();

        driver = new GamepadEx(gamepad1);

        telemetry.addLine("init done");
        telemetry.update();
    }

    //TODO: test if this works, if not remove
    private void applyPIDF() {
        double kFLocal = kF;
        if (kFLocal == 0.0) {
            // Based on achievable max ticks/s reported by the SDK
            // Good starting point for later tuning in Dashboard.
            double maxTps = shooter1.getMotorType().getAchieveableMaxTicksPerSecond();
            if (maxTps <= 0) maxTps = (1620.0 * TICKS_PER_REV) / 60.0;
            // REV internal scaling expects kF around 32767/maxVelocity as a reasonable baseline
            kFLocal = 32767.0 / maxTps;
        }
        kfValue = kFLocal;
        shooter1.setVelocityPIDFCoefficients(kP, kI, kD, kFLocal);
        shooter2.setVelocityPIDFCoefficients(kP, kI, kD, kFLocal);
    }

    @Override
    public void run() {
        super.run();

        /** Live PIDF updates via dashboard */
        applyPIDF();

        double targetMotorRPM = TARGET_RPM / GEAR_RATIO;
        double targetTicksPerSec = (targetMotorRPM * TICKS_PER_REV) / 60.0;

        if (runShooter) {
            shooter1.setVelocity(targetTicksPerSec); // ticks/s
            shooter2.setVelocity(targetTicksPerSec);
        } else {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }

        // Gets current velo of motor
        double currTicksPerSec = shooter1.getVelocity(); // ticks/s of motor
        double currTicksPerSec2 = shooter2.getVelocity();
        double currMotorRPM = (((currTicksPerSec + currTicksPerSec2) / 2) * 60) / TICKS_PER_REV;
        double currShooterRPM = currMotorRPM * GEAR_RATIO;

        // Tuning Stuff
        double errorMotorRPM = targetMotorRPM - currMotorRPM;
        double errorShooterRPM = TARGET_RPM - currShooterRPM;

        // ===== Graphs on FTC Dashboard =====
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("target_shooter_rpm", TARGET_RPM);
        packet.put("current_shooter_rpm", currShooterRPM);
        packet.put("target_motor_rpm", targetMotorRPM);
        packet.put("current_motor_rpm", currMotorRPM);
        packet.put("error_shooter_rpm", errorShooterRPM);
        packet.put("error_motor_rpm", errorMotorRPM);
        packet.put("motor_ticks_per_sec", currTicksPerSec);
        packet.put("kF Local Value: ", kfValue);
        dash.sendTelemetryPacket(packet);
    }
}
