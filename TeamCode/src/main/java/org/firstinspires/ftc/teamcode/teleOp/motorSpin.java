package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class motorSpin extends CommandOpMode {
    public DcMotorEx motor1;

    @Override
    public void initialize() {
        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
    }

    @Override
    public void run() {
        super.run();

        motor1.setPower(1);
    }
}
