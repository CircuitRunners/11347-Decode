package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class intake extends SubsystemBase {
    public DcMotorEx intakeRoller;
    public DcMotorEx transferMech;
    public static boolean intake = true;
    public static boolean transfer = true;

    public intake(HardwareMap hardwareMap) {
        intakeRoller = hardwareMap.get(DcMotorEx.class, "intake");
        transferMech = hardwareMap.get(DcMotorEx.class, "transfer");
    }

    public void runIntake(double power) {
        double direction = (intake ? power : -power);

        intakeRoller.setPower(-power);
    }

    public void setTransfer(double power) {
        double direction = (transfer ? power : -power);

        transferMech.setPower(power);
    }
}
