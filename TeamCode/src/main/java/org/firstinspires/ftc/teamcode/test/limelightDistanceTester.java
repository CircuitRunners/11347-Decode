package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.*;
@TeleOp
public class limelightDistanceTester extends OpMode{
    private Limelight3A limelight3A;
    private double distance;

    private IMU imu;

    @Override
    public void init() {

        limelight3A = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight3A.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start(){
        limelight3A.start();
    }

    @Override
    public void loop() {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult !=null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            Pose follower = getCamPose();
            String followerData = String.format(Locale.US,
                    "{X: %.3f, Y: %.3f, H: %.3f}",
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    Math.toDegrees(follower.getPose().getHeading())

            );

            distance = getDistanceFromTag(llResult.getTa());
            telemetry.addData("Calculated Distance", distance);
            telemetry.addData("Tx", llResult.getTx());
            //telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Botpose", botPose.toString());
            telemetry.addData("Follower Position:", followerData);
        }

    }


    public double getDistanceFromTag(double ta){
        double scale = 383.3941;
        double distance = Math.pow(scale / ta, 1/1.427875);
        return distance;

    }

    public Pose getCamPose() {

            Pose3D botpose = limelight3A.getLatestResult().getBotpose_MT2();
            Pose3D botpose2 = limelight3A.getLatestResult().getBotpose();

            Position poseIn = botpose.getPosition().toUnit(DistanceUnit.INCH);

            Pose limelightPose = new Pose(poseIn.x, poseIn.y, botpose2.getOrientation().getYaw());
            Pose pedroPose = new Pose(limelightPose.getX(), limelightPose.getY(), limelightPose.getHeading());


            return pedroPose;

    }

}
