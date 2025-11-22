package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp
public class outtakeTest extends CommandOpMode {
    public DcMotorEx spin;
    public Servo hood;
    public CRServo turretLeft;
    public CRServo turretRight;
    public static double power = 1;

    @Override
    public void initialize() {
        spin = hardwareMap.get(DcMotorEx.class, "spin");
        hood = hardwareMap.get(Servo.class, "hood");
        turretLeft = hardwareMap.get(CRServo.class, "turretLeft");
        turretRight = hardwareMap.get(CRServo.class, "turretRight");


    }

    @Override
    public void run() {
        super.run();

        spin.setPower(1);
        if(gamepad1.dpad_up) {
            hood.setPosition(hood.getPosition()+.01);
        } else if (gamepad1.dpad_down) {
            hood.setPosition(hood.getPosition()-.01);
        }

        if(gamepad1.dpad_right) {
            turretLeft.setPower(1);
            turretRight.setPower(1);
        } else if (gamepad1.dpad_left) {
            turretLeft.setPower(-1);
            turretRight.setPower(-1);
        }

    }
}
