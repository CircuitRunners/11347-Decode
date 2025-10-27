package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@Disabled
@TeleOp
public class motorSpin extends CommandOpMode {
    public DcMotorEx motor1;
    public static double power = 1;

    @Override
    public void initialize() {
        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
    }

    @Override
    public void run() {
        super.run();

        motor1.setPower(power);
    }
}
