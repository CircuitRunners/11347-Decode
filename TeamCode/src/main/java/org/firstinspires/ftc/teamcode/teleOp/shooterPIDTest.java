package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class shooterPIDTest extends CommandOpMode {
    private DcMotorEx shooter;
    public GamepadEx driver;

    // Dashboard-tunable constants
    public static double TARGET_RPM = 4000.0;
    public static double MOTOR_RPM = 1620.0;
    public static double GEAR_RATIO = 2.5;
    public static double TICKS_PER_REV = 103.8;

    public static boolean runShooter = false;

    @Override
    public void initialize() {
        shooter = hardwareMap.get(DcMotorEx.class, "m1");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driver = new GamepadEx(gamepad1);
        telemetry.addLine("init done");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        double motorRPM = MOTOR_RPM;
        double targetTicksPerSec = (motorRPM * TICKS_PER_REV) / 60.0;

        if (gamepad1.right_bumper) {
            runShooter = !runShooter;
        }

        if (runShooter) {
            shooter.setVelocity(targetTicksPerSec);
        } else {
            shooter.setVelocity(0);
        }

        double currTicksPerSec = shooter.getVelocity(); // ticks/s (motor)
        double currMotorRPM    = (currTicksPerSec * 60.0) / TICKS_PER_REV;
        double currShooterRPM  = currMotorRPM * GEAR_RATIO;

        telemetry.addData("Target Shooter RPM", TARGET_RPM);
        telemetry.addData("Target Motor RPM", motorRPM);
        telemetry.addData("Target Ticks/s", targetTicksPerSec);

        telemetry.addData("Curr Motor RPM", "%.1f", currMotorRPM);
        telemetry.addData("Curr Shooter RPM (est.)", "%.1f", currShooterRPM);
        telemetry.addData("Curr Ticks/s", "%.0f", currTicksPerSec);
        telemetry.update();
    }
}
