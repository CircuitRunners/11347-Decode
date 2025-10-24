package org.firstinspires.ftc.teamcode.teleOp.competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.mecanumDB;
import org.firstinspires.ftc.teamcode.subsystems.outtake;
import org.firstinspires.ftc.teamcode.support.SRSHub;

import org.firstinspires.ftc.teamcode.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;

@TeleOp(group="1")
public class MainTeleOp extends CommandOpMode {
    private StaticShooter shooter;
    private mecanumDB drive;
    private outtake out;
    private intake in;
    private SRSHub srs;

    // Stuff for resetting pinpoint location
    private float xOffset = 0, yOffset = 0, headingOffset = 0;
    private static int shooterSpeed = 0;

    private GamepadEx driver, manipulator;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);

        SRSHub.Config config = new SRSHub.Config();
        config.addI2CDevice(
                1,
                new SRSHub.GoBildaPinpoint(
                        -28.042f,
                        -147.02f,
                        19.89436789f,
                        SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD,
                        SRSHub.GoBildaPinpoint.EncoderDirection.REVERSED
                )
        );

        RobotLog.clearGlobalWarningMsg();
        srs = hardwareMap.get(SRSHub.class, "srsHub");
        srs.init(config);

        while (!srs.ready()) {
            idle();
        }
        srs.update();

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);
        drive = new mecanumDB(hardwareMap);
        out = new outtake(hardwareMap);
        in = new intake(hardwareMap);


        shooterSpeed = 0;

        // Default Commands
        // Intake Command
        in.setDefaultCommand(new IntakeCommand(in, out, driver));

        // Shooting
        // Click bumper once to activate intake at full speed
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(()-> shooter.setTargetRPM(3500)));

        // Click bumper once to activate intake at close speed
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(()-> shooter.setTargetRPM(2500)));

        // Click both bumpers to turn shooter off
        Trigger shooterOff = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER));
        shooterOff.whenActive(new InstantCommand(() -> shooter.setTargetRPM(0)));

        // Transfering Command
        // Click to toggle on and off transfering
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(new TransferCommand(in, out, driver));

        telemetry.addLine("ROBOT READY!");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        srs.update();
        shooter.runShooter(); // shouldn't run by default

        double forward = driver.getLeftY(); // Forwards/backwards
        double right = driver.getLeftX(); // Strafe
        double rotate = driver.getRightX(); // Rotation

        Pose2D pose = driveFieldRelative(srs, forward, right, rotate);
        SRSHub.GoBildaPinpoint pinpoint = srs.getI2CDevice(1, SRSHub.GoBildaPinpoint.class);
        Pose2D relativePose = getPinpointPose(pinpoint);

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(()-> {
                    resetPinpoint(pinpoint);
                }));

        out.aiming(gamepad1.dpad_up,
                gamepad1.dpad_down);

        // --- Telemetry ---
        telemetry.addData("Shooter Encoder Vel", shooter.getShooterVelocity());
        telemetry.addData("Aiming Servo Pos: ", out.getAimPos());

        telemetry.addData("Pinpoint X (mm)", relativePose.getX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y (mm)", relativePose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", Math.toDegrees(relativePose.getHeading(AngleUnit.RADIANS)));

        telemetry.addData("X Vel (mm/s)", pinpoint.xVelocity);
        telemetry.addData("Y Vel (mm/s)", pinpoint.yVelocity);
        telemetry.addData("H Vel (rad/s)", pinpoint.hVelocity);
        telemetry.update();
    }

    private Pose2D driveFieldRelative(SRSHub hub, double forward, double right, double rotate) {
        // Update all SRS-connected devices
        hub.update();

        SRSHub.GoBildaPinpoint pinpoint = hub.getI2CDevice(1, SRSHub.GoBildaPinpoint.class);

        // Get live pose data
        double x = pinpoint.xPosition;
        double y = pinpoint.yPosition;
        double heading = pinpoint.hOrientation; // radians

        Pose2D pos = new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);

        // Field-centric transform
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - heading);

        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        drive.drive(newForward, newRight, rotate);
        return pos;
    }

    public void resetPinpoint(SRSHub.GoBildaPinpoint pinpoint) {
        xOffset = pinpoint.xPosition;
        yOffset = pinpoint.yPosition;
        headingOffset = pinpoint.hOrientation;
    }

    public Pose2D getPinpointPose(SRSHub.GoBildaPinpoint pinpoint) {
        double x = pinpoint.xPosition - xOffset;
        double y = pinpoint.yPosition - yOffset;
        double heading = pinpoint.hOrientation -AngleUnit.normalizeRadians(headingOffset);
        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
    }
}
