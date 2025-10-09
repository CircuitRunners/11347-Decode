package org.firstinspires.ftc.teamcode.teleOp;

/*intake = right trigger
outtake= a
drive= forward/backward/left/right (left joystick)
drive= turning (right joystick)
servo = bumper
*/


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.mecanumDB;

@Config
@TeleOp
public class teleOp1 extends CommandOpMode {
    public mecanumDB drivebase;
    public static double power = 1;

    @Override
    public void initialize() {
        drivebase = new mecanumDB(hardwareMap);
    }

    @Override
    public void run() {
        super.run();

        drivebase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}


