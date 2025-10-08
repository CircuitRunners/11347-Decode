package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake extends SubsystemBase {
        public DcMotorEx intakeRoller;
        public DcMotorEx transferMech;



        public intake(HardwareMap hardwareMap) {
            intakeRoller = hardwareMap.get(DcMotorEx.class, "intake");
            transferMech = hardwareMap.get(DcMotorEx.class, "transfer");

            }
    public void runIntake(double forward) {
        intakeRoller.setPower(forward);
    }
    public void setTransferMech(double power) {
            transferMech.setPower(power);
    }
        }
