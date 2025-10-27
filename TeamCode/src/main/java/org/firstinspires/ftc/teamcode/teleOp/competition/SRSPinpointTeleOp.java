package org.firstinspires.ftc.teamcode.teleOp.competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.support.SRSHub;
import org.firstinspires.ftc.teamcode.subsystems.mecanumDB;

@Deprecated
@Disabled
@TeleOp(name="SRS Pinpoint teleOp", group="1")
public class SRSPinpointTeleOp extends CommandOpMode {
    private SRSHub srs;
    private mecanumDB drive;

    private GamepadEx driver, manipulator;

    // Software offsets for "zeroing" the Pinpoint
    private float xOffset = 0, yOffset = 0, headingOffset = 0;

    @Override
    public void initialize() {
        // Use both Driver Station and Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Clear REV Hub cache to avoid stale reads
        schedule(new BulkCacheCommand(hardwareMap));

        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);
        drive = new mecanumDB(hardwareMap);

        // --- Configure SRS Hub ---
        SRSHub.Config config = new SRSHub.Config();

        // Encoder 1 (flywheel) on port E1
        config.setEncoder(1, SRSHub.Encoder.QUADRATURE);

        // goBILDA Pinpoint on I2C Port 1
        config.addI2CDevice(
                1,
                new SRSHub.GoBildaPinpoint(
                        28.042f,   // x offset (mm, right = +)
                        -147.012f, // y offset (mm, behind = -)
                        19.89f,
                        SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD,
                        SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD
                )
        );

        RobotLog.clearGlobalWarningMsg();

        // Get the hub from hardwareMap
        srs = hardwareMap.get(SRSHub.class, "srsHub");
        srs.init(config);

        // Wait until SRS Hub firmware is ready
        while (!srs.ready()) {
            idle();
        }

        srs.update();

        telemetry.addLine("Init Done");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        srs.update();

        // Read gamepad inputs
        double forward = driver.getLeftY(); // Forward stick (Y)
        double right   = driver.getLeftX();  // Strafe stick (X)
        double rotate  = driver.getRightX(); // Rotation stick (X)

        // Drive with field-centric control
        Pose2D pose = driveFieldRelative(srs, forward, right, rotate);

        // Get pinpoint data for telemetry
        SRSHub.GoBildaPinpoint pinpoint = srs.getI2CDevice(1, SRSHub.GoBildaPinpoint.class);
        Pose2D relativePose = getPinpointPose(pinpoint);

        // --- Telemetry ---
        telemetry.addData("Shooter Encoder Vel", srs.readEncoder(1).velocity);

        telemetry.addData("Pinpoint X (mm)", relativePose.getX(DistanceUnit.MM));
        telemetry.addData("Pinpoint Y (mm)", relativePose.getY(DistanceUnit.MM));
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
