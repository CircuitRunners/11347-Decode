package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.support.SRSHub;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.1)
            .forwardZeroPowerAcceleration(-29.64945897478038)
            .lateralZeroPowerAcceleration(-53.7580800784387)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.025, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.02, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.012, 0, 0.001, 0.6, 0.025))
            .centripetalScaling(0.0007);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            0.7,
            1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(1.6704066118856116)
            .yVelocity(3.927147842767667);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-28.042) //-147.012
            .strafePodX(-147.012) //28.042
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setLocalizer(new SRSLocalizer(
                        hardwareMap,
                        1,
                        -28.042f,
                        -147.012f,
                        19.89436789f,
                        SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD,
                        SRSHub.GoBildaPinpoint.EncoderDirection.REVERSED,
                        0
                ))
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

//    public static Follower createFollower(HardwareMap hardwareMap) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .pinpointLocalizer(localizerConstants)
//                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(driveConstants)
//                .build();
//    }
}
