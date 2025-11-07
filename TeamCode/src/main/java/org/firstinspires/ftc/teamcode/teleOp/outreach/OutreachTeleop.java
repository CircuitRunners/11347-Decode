package org.firstinspires.ftc.teamcode.teleOp.outreach;

/*intake = right trigger
outtake= a
drive= forward/backward/left/right (left joystick)
drive= turning (right joystick)
servo = bumper
... no
*/

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivebase;

@Config
@Disabled
@TeleOp
public class OutreachTeleop extends CommandOpMode {
    public MecanumDrivebase drivebase;
    GamepadEx driver;
    GamepadEx manipulator;
    private OuttakeSubsystem outtake;
    private IntakeSubsystem intake;
    private double outtakeMotor = 0;
    private double transferMotor = 0;
    private double intakeMotor = 0;
    private int outtakePosition = -1;
    private int intakePosition = -1;
    private int transferPosition = -1;

    @Override
    public void initialize() {
        drivebase = new MecanumDrivebase(hardwareMap);
        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);
        outtake = new OuttakeSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);




        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                .whileActiveContinuous(new InstantCommand(() -> {
                    intakeMotor = 1;
                }));
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whileActiveContinuous(new InstantCommand(() -> {
                    intakeMotor = -0.67;
                }));



        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    transferPosition = (transferPosition +1) % 2;
                }));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    outtakePosition += 1;
                }));


        telemetry.addLine("READY!");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        if (gamepad1.left_trigger > 0.1) {
            intakeMotor = 1;
        }
        else {
            intakeMotor = 0;
        }

        if (gamepad1.right_trigger > 0.1) {
            intakeMotor = -0.6;
        }
        else {
            intakeMotor = 0;
        }





        drivebase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        switch (transferPosition) {
            case 0:
                if (transferMotor <1) {
                    transferMotor = -0.5; //might need to be reversed idk
                }
                break;
            case 1:
                if (transferMotor > 0) {
                    transferMotor = 1;
                }
                break;
            default:
                break;
        }


        switch (outtakePosition) {
            case 0:
                if (outtakeMotor <1) {
                    outtakeMotor = 1;
                }
                break;
            case 1:
                if (outtakeMotor > 0) {
                    outtakeMotor = 0;
                    outtakePosition = -1;
                }
                break;
            default:
                break;
        }


    }
}


