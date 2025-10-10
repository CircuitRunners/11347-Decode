package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class intake extends SubsystemBase {
    public DcMotorEx intakeRoller;
    public DcMotorEx transferMech;

    public static boolean intakeDirection = true;
    public static boolean transferDirection = true;

    public intake(HardwareMap hardwareMap) {
        intakeRoller = hardwareMap.get(DcMotorEx.class, "intake");
        transferMech = hardwareMap.get(DcMotorEx.class, "transfer");

    }

    public void runIntake(double forward) {
        double power = forward;
        if (!intakeDirection){
            power = -forward;
        }

        intakeRoller.setPower(power);
    }

    public void setTransferMech(double forward) {
        double power = forward;
        if(!transferDirection){
            power = - forward;
        }

        transferMech.setPower(power);
    }
}

