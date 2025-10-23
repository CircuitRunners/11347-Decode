package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.support.SRSHub;

/**
 * Localizer that uses an SRSHub-connected goBILDA Pinpoint for robot pose tracking.
 * Compatible with Pedro Pathing Follower.
 */
public class SRSLocalizer implements Localizer {
    private final SRSHub hub;
    private final SRSHub.GoBildaPinpoint pinpoint;

    // Software offsets for zeroing the position
    private double xOffset = 0;
    private double yOffset = 0;
    private double headingOffset = 0;

    // Multipliers (used for scaling tuning results)
    private double forwardMultiplier = 1.0;
    private double lateralMultiplier = 1.0;
    private double turningMultiplier = 1.0;

    private double headingScalar = 1.0;      // tune this
    private double totalHeading = 0.0;       // continuous (unwrapped) heading, radians
    private double lastRawHeading = Double.NaN; // for unwrap

    private Pose currentPose = new Pose();
    private Pose velocity = new Pose();

    public SRSLocalizer(HardwareMap hardwareMap,
                        int i2cPort,
                        float srsPinpointXOffset,
                        float srsPinpointYOffset,
                        float encoderResolution,
                        SRSHub.GoBildaPinpoint.EncoderDirection xEncoderDirection,
                        SRSHub.GoBildaPinpoint.EncoderDirection yEncoderDirection,
                        double headingOffsetDeg
    ) {
        SRSHub.Config config = new SRSHub.Config();

        config.addI2CDevice(
                i2cPort,
                new SRSHub.GoBildaPinpoint(
                        srsPinpointXOffset,   // X offset (mm)
                        srsPinpointYOffset,   // Y offset (mm)
                        encoderResolution,
                        xEncoderDirection,
                        yEncoderDirection
                )
        );

        hub = hardwareMap.get(SRSHub.class, "srsHub");
        hub.init(config);

        while (!hub.ready()) Thread.yield();

        pinpoint = hub.getI2CDevice(i2cPort, SRSHub.GoBildaPinpoint.class);
        this.headingOffset = headingOffsetDeg;
    }

    @Override
    public void update() {
        hub.update();

        double x = pinpoint.xPosition;
        double y = pinpoint.yPosition;
        double rawHeading = pinpoint.hOrientation - headingOffset;

        if (Double.isNaN(lastRawHeading)) {
            lastRawHeading = rawHeading;
        }

        double delta = AngleUnit.normalizeRadians(rawHeading - lastRawHeading);
        lastRawHeading = rawHeading;

        totalHeading += delta * headingScalar;

        // Convert mm → inches and apply tuning multipliers
        currentPose = new Pose(
                DistanceUnit.MM.toInches(x) * forwardMultiplier,
                DistanceUnit.MM.toInches(y) * lateralMultiplier,
                AngleUnit.normalizeRadians(
                        totalHeading) * turningMultiplier
        );

        // Convert velocities to inches/s (still robot-relative)
        velocity = new Pose(
                DistanceUnit.MM.toInches(pinpoint.yVelocity),
                DistanceUnit.MM.toInches(pinpoint.xVelocity),
                pinpoint.hVelocity * headingScalar
        );
    }

    @Override
    public Pose getPose() {
        return currentPose;
    }

    @Override
    public Pose getVelocity() {
        return velocity;
    }

    @Override
    public Vector getVelocityVector() {
        return new Vector(
                Math.hypot(velocity.getX(), velocity.getY()),
                Math.atan2(velocity.getY(), velocity.getX())
        );
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return forwardMultiplier;
    }

    @Override
    public double getLateralMultiplier() {
        return lateralMultiplier;
    }

    @Override
    public double getTurningMultiplier() {
        return turningMultiplier;
    }

    @Override
    public void resetIMU() throws InterruptedException {
        reset();
    }

    @Override
    public double getIMUHeading() {
        return currentPose.getHeading();
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(currentPose.getX()) ||
                Double.isNaN(currentPose.getY()) ||
                Double.isNaN(currentPose.getHeading());
    }

    @Override
    public void setStartPose(Pose startPose) {
        xOffset = pinpoint.xPosition - DistanceUnit.INCH.toMm(startPose.getX());
        yOffset = pinpoint.yPosition - DistanceUnit.INCH.toMm(startPose.getY());
        headingOffset = pinpoint.hOrientation - startPose.getHeading();
    }

    @Override
    public void setPose(Pose pose) {
        currentPose = pose;
    }

    public void reset() {
        xOffset = pinpoint.xPosition;
        yOffset = pinpoint.yPosition;
        headingOffset = pinpoint.hOrientation;
    }

    public void setMultipliers(double forward, double lateral, double turning) {
        this.forwardMultiplier = forward;
        this.lateralMultiplier = lateral;
        this.turningMultiplier = turning;
    }
}
