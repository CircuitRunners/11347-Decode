package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class staticShooter {
    private DcMotorEx shooter;
    private GamepadEx driver;

    // Dashboard-tunable constants
    public static double TARGET_RPM = 3500.0; // 4000
    public static double MOTOR_RPM = 1620.0; // 1410
    public static double GEAR_RATIO = 2.5;
    public static double TICKS_PER_REV = 103.8;

    // PIDF (velocity)
    public static double kP = 35.0;
    public static double kI = 0.0;
    public static double kD = 10.0;
    public static double kF = 13.0;

    public double kfValue;

    private FtcDashboard dash;
    private Telemetry telemetry;

    public staticShooter(HardwareMap hardwareMap) {
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyPIDF();

        telemetry.addLine("shooter Init Done");
    }

    public void setShooterPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
    }

    private void applyPIDF() {
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

//    public
}
