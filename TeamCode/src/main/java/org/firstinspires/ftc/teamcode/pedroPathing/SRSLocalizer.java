package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.support.SRSHub;

import java.util.Objects;

public class SRSLocalizer implements Localizer {
    private final SRSHub hub;
    private final SRSHub.GoBildaPinpoint pinpoint;
    private double previousHeading = 0;
    private double totalHeading = 0;
    private Pose startPose;
    private Pose currentVelocity;
    private Pose pinpointPose;

    public SRSLocalizer(HardwareMap map, SRSConstants constants) { this(map, constants, new Pose()); }

    public SRSLocalizer(HardwareMap map, SRSConstants constants, Pose startPose) {
        SRSHub.Config config = new SRSHub.Config();
        config.addI2CDevice(
                constants.i2cPort,
                new SRSHub.GoBildaPinpoint(
                        constants.forwardPodX,
                        constants.strafePodY,
                        constants.encoderResolution,
                        constants.forwardEncoderDirection,
                        constants.strafeEncoderDirection));

        hub = map.get(SRSHub.class, constants.hardwareMapName);
        hub.init(config);
        while (!hub.ready()) Thread.yield();
        pinpoint = hub.getI2CDevice(constants.i2cPort, SRSHub.GoBildaPinpoint.class);

        totalHeading = 0;
        pinpointPose = startPose;
        currentVelocity = new Pose();
        previousHeading = startPose.getHeading();
    }

    @Override
    public void update() {
        hub.update();

        Pose currentPinpointPose = PoseConverter.pose2DToPose(getPosition(), PedroCoordinates.INSTANCE);
        totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), previousHeading) * MathFunctions.getTurnDirection(previousHeading, currentPinpointPose.getHeading());
        previousHeading = currentPinpointPose.getHeading();
        currentVelocity = new Pose(
                DistanceUnit.MM.toInches(pinpoint.xVelocity),
                -DistanceUnit.MM.toInches(pinpoint.yVelocity),
                pinpoint.hVelocity);
        pinpointPose = currentPinpointPose;
    }

    private Pose2D getPosition() {
        return new Pose2D(DistanceUnit.MM,
                pinpoint.xPosition,
                pinpoint.yPosition,
                AngleUnit.RADIANS,
                AngleUnit.normalizeRadians(pinpoint.hOrientation));
    }

    @Override
    public Pose getPose() {
        return pinpointPose;
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        hub.update();

        if (!Objects.equals(startPose, new Pose()) && startPose != null) {
            Pose currentPose = pinpointPose.rotate(-startPose.getHeading(), false).minus(startPose);
            setPose(setStart.plus(currentPose.rotate(setStart.getHeading(), false)));
        } else {
            setPose(setStart);
        }

        this.startPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        pinpointPose = setPose;
        previousHeading = setPose.getHeading();
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return 1.0;
    }

    @Override
    public double getLateralMultiplier() {
        return 1.0;
    }

    @Override
    public double getTurningMultiplier() {
        return 1.0;
    }

    @Override
    public void resetIMU() {
        reset();
    }

    @Override
    public double getIMUHeading() {
        return pinpointPose.getHeading();
    }

    public void reset() {}

    @Override
    public boolean isNAN() {
        return Double.isNaN(pinpointPose.getX())
                || Double.isNaN(pinpointPose.getY())
                || Double.isNaN(pinpointPose.getHeading());
    }
}
