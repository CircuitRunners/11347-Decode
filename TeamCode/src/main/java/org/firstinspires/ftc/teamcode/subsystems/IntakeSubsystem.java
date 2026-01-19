package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class IntakeSubsystem extends SubsystemBase {

    public DcMotorEx intakeRoller;
    public DcMotorEx transferMech;
    public static boolean intake = true;
    public static boolean transfer = true;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeRoller = hardwareMap.get(DcMotorEx.class, "intake");
        transferMech = hardwareMap.get(DcMotorEx.class, "transfer");
    }

    public void runIntake(double power) {
        double direction = (intake ? power : -power);

        intakeRoller.setPower(power);
    }

    public void runTransfer(double power) {
        double direction = (transfer ? power : -power);

        transferMech.setPower(power);
    }

    public void shoot() {
        runTransfer(1);
        runIntake(0.5);
    }

    public void intaking(double power) {
        runIntake(power);
        runTransfer(-0.5);
    }

    public void transfer() {
        runIntake(0.85);
        runTransfer(1);
    }

    public void stop() {
        runTransfer(0);
        runIntake(0);
    }
}
