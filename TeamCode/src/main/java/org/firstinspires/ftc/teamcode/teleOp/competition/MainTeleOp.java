package org.firstinspires.ftc.teamcode.teleOp.competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
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

import java.util.function.BooleanSupplier;

@TeleOp(group="1")
public class MainTeleOp extends CommandOpMode {
    private StaticShooter shooter;
    private mecanumDB drive;
    private outtake out;
    private intake in;
    private SRSHub srs;

    // Stuff for resetting pinpoint location
    private float xOffset = 0, yOffset = 0, headingOffset = 0;

    private GamepadEx driver, manipulator;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);

        SRSHub.Config config = new SRSHub.Config();
        config.setEncoder(1, SRSHub.Encoder.QUADRATURE);
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


        // --- Shooter Control ---
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            shooter.setTargetRPM(3500);
                        }),
                        new WaitUntilCommand(()-> {
                            double velocity = srs.readEncoder(1).velocity;
                            return velocity > 3400 && velocity < 3600;
                        }),
                        new InstantCommand(out::unblock),
                        new WaitCommand(500),
                        new InstantCommand(in::shoot)
                ))
                .whenReleased(new SequentialCommandGroup(
                        new InstantCommand(()-> {
                            in.stop();
                            out.block();
                        })
                ));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            shooter.setTargetRPM(2500);
                        }),
                        new WaitUntilCommand(()-> {
                            double velocity = srs.readEncoder(1).velocity;
                            return velocity > 2500 && velocity < 2700;
                        }),
                        new InstantCommand(out::unblock),
                        new WaitCommand(500),
                        new InstantCommand(in::shoot)
                ))
                .whenReleased(new SequentialCommandGroup(
                        new InstantCommand(()-> {
                            in.stop();
                            out.block();
                        })
                ));

        Trigger shooterOff = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER));
        shooterOff.whenActive(new InstantCommand(() -> {
            shooter.setTargetRPM(0);
            out.block();
        }));

        // --- Intake Control ---
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 ||
                driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whileActiveContinuous(new InstantCommand(()-> {
                    double leftTrigger = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                    double rightTrigger = manipulator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

                    out.block();
                    in.runIntake(rightTrigger - leftTrigger);
                    in.runTransfer(-0.2);
                }))
                .whenInactive(new InstantCommand(()-> {
                    in.stop();
                    out.block();
                }));

        telemetry.addLine("ROBOT READY!");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        srs.update();
        shooter.runShooter(); // shouldnt run by default

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

        // --- Telemetry ---
        telemetry.addData("Shooter Encoder Vel", srs.readEncoder(1).velocity);

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
        double heading = AngleUnit.normalizeRadians(pinpoint.hOrientation - headingOffset);
        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
    }
}
