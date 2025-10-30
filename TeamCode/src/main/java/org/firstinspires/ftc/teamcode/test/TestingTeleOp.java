package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@Disabled
@TeleOp
public class TestingTeleOp extends CommandOpMode {
    private StaticShooter shooter;
    private MecanumDrivebase db;
    private OuttakeSubsystem out;
    private IntakeSubsystem in;

    private boolean activeFast = false;
    private boolean activeSlow = false;

    private GamepadEx driver;

    @Override
    public void initialize() {
        shooter = new StaticShooter(hardwareMap, telemetry);
        db = new MecanumDrivebase(hardwareMap);
        out = new OuttakeSubsystem(hardwareMap);
        in = new IntakeSubsystem(hardwareMap);

        driver = new GamepadEx(gamepad1);

        telemetry.addLine("Init Done!");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        shooter.applyPIDF();
        shooter.update();

        double forward = driver.getRightY();
        double strafe = driver.getRightX();
        double rotate = driver.getLeftX();

        db.drive(-forward, strafe, rotate);

        in.runIntake(driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        in.runTransfer(driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        if (driver.getButton(GamepadKeys.Button.RIGHT_BUMPER) && activeFast) {
            shooter.setTargetRPM(3500);
            activeFast = !activeFast;
        } else if (driver.getButton(GamepadKeys.Button.RIGHT_BUMPER) && !activeFast) {
            shooter.setTargetRPM(0);
            activeFast = !activeFast;
        }

        if (driver.getButton(GamepadKeys.Button.LEFT_BUMPER) && activeSlow) {
            shooter.setTargetRPM(1000);
            activeSlow = !activeSlow;
        } else if (driver.getButton(GamepadKeys.Button.LEFT_BUMPER) && !activeSlow) {
            shooter.setTargetRPM(0);
            activeSlow = !activeSlow;
        }


        telemetry.addLine("Shooter Velocity: " + shooter.getShooterVelocity());
        telemetry.update();
    }
}
