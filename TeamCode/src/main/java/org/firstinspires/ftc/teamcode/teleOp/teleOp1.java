package org.firstinspires.ftc.teamcode.teleOp;

/*intake = right trigger
outtake= a
drive= forward/backward/left/right (left joystick)
drive= turning (right joystick)
servo = bumper
*/


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.mecanumDB;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Config
@TeleOp
public class teleOp1 extends CommandOpMode {
    public mecanumDB drivebase;
    public Intake intake;
    public Outtake outtake;
    //public static double power = 1;
    public Timer outtakeTimer;
    public Timer blockTimer;


    @Override
    public void initialize() {
        drivebase = new mecanumDB(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        outtakeTimer = new Timer();
        blockTimer = new Timer();
    }

    @Override
    public void run() {
        super.run();

        drivebase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        intake.runIntake(gamepad1.right_trigger);
        intake.setTransferMech(gamepad1.left_trigger);
        if (gamepad1.a && outtakeTimer.getElapsedTime()>=200) {
            outtakeTimer.resetTimer();
            outtake.toggleOuttake();
        }
        outtake.aiming(gamepad1.dpad_up, gamepad1.dpad_down);
        //TODO: blocking, not required but helpful
        /*
        if (gamepad1.dpad_right && blockTimer.getElapsedTime()>=200) {
            blockTimer.resetTimer();
            outtake.setBlock();
        } else if (gamepad1.dpad_left && blockTimer.getElapsedTime()>=200) {
            blockTimer.resetTimer();
            outtake.nonBlock();
        }*/
        telemetry.addData("Left Block Position", outtake.blockingServo.getPosition());
        telemetry.addData("Right Block Position", outtake.blockingServoTwo.getPosition());
        telemetry.update();
    }
}


