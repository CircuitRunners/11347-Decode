package org.firstinspires.ftc.teamcode.test;

/*intake = right trigger
outtake= a
drive= forward/backward/left/right (left joystick)
drive= turning (right joystick)
servo = bumper
*/


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivebase;

@Disabled
@Config
@TeleOp
public class teleOp1 extends CommandOpMode {
    public MecanumDrivebase drivebase;
    public static double power = 1;

    @Override
    public void initialize() {
        drivebase = new MecanumDrivebase(hardwareMap);
    }

    @Override
    public void run() {
        super.run();

        drivebase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}


