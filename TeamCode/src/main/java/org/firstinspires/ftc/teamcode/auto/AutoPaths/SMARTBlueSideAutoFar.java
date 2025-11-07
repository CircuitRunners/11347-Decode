package org.firstinspires.ftc.teamcode.auto.AutoPaths;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamAprilTag;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;

import java.util.List;

@Disabled
@Config
@Configurable
@Autonomous(name="Smart Blue Side Auto Far",group="Blue Autos")
public class SMARTBlueSideAutoFar extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    Timer shootTime = new Timer();
    private StaticShooter shooter;
    private IntakeSubsystem in;
    private OuttakeSubsystem out;
    private LimelightSubsystem limelight;
    private WebcamAprilTag webcam;

    private boolean intaking, transfering, scoring, moving;

    private boolean headingLockEnabled;

    private final Pose startPose = new Pose(40.0, 8.2, Math.toRadians(180));

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);
        out = new OuttakeSubsystem(hardwareMap);
        in = new IntakeSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, "limelight");
        AlliancePresets.setAllianceShooterTag(AlliancePresets.Alliance.BLUE.getTagId());
        limelight.setAllianceTagID(AlliancePresets.getAllianceShooterTag());
        webcam = new WebcamAprilTag(hardwareMap, "obeliskCam");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        follower.update();

        buildPaths();
        pathTimer = new Timer();
        pathTimer.resetTimer();

        AlliancePresets.setAllianceShooterTag(AlliancePresets.Alliance.BLUE.getTagId());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        webcam.detectDuringInit();
        telemetry.addData("Tag Of Interest:", webcam.getDetectedTag());
        telemetry.addData("Pinpoint X", follower.getPose().getX());
        telemetry.addData("Pinpoint Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(-2);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        limelight.update();
        autonomousPathUpdate();

//        if (headingLockEnabled && limelight.hasValidTarget()) {
//            LLResult result = limelight.getLatest();
//            if (result != null && result.isValid()) {
//                double finalRotation = result.getTxNC() * 0.02;
//                finalRotation = Math.max(-0.4, Math.min(finalRotation, 0.4));
//                follower.setRotation(finalRotation);
//            }
//        }

        telemetry.addData("Follower busy?", follower.isBusy());
        telemetry.addData("Path State: ", pathState);
        telemetry.addData("Shooter Velo: ", shooter.getShooterVelocity());
        telemetry.addData("Timer: ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading (deg)", "%.1f", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    private void autonomousPathUpdate() {
        switch (AlliancePresets.getCurrentCypher()) {
            case 21:
                switch (pathState) {
                    case -2:
                        if (!follower.isBusy()) {
                            shooter.setTargetRPM(3500);
                            out.aimScoring();
                            setPathState(0);
                        }
                        break;

                    case 0:
                        if (!follower.isBusy()) {
                            follower.followPath(line1GPP);
                            shootTime.resetTimer();
                            setPathState(-1);
                        }
                        break;

                    case -1:


                        if (shootTime.getElapsedTimeSeconds() < 15) {
                            if (shooter.getShooterVelocity() >= 3250 && shooter.getShooterVelocity() < 3450) {
                                transfer();

                            } else if (shooter.getShooterVelocity() < 3250) {
                                stopTransfer();

                            }
                        } else {
                            stopTransfer();
                            shooter.setTargetRPM(0);
                            setPathState(1);
                        }

                        break;

                    case 1:
                        if (!follower.isBusy()) {
                            intake();                       // replaces in.runIntake(1);
                            follower.followPath(line2GPP);
                            setPathState(2);
                        }
                        break;

                    case 2:
                        if (!follower.isBusy()) {
                            follower.followPath(line3GPP);
                            //setPathState(3);
                        }
                        break;

                    case 3:
                        if (!follower.isBusy()) {
                            follower.followPath(line4GPP);
                            setPathState(4);
                        }
                        break;

                    case 4:
                        if (!follower.isBusy()) {
                            follower.followPath(line5GPP);
                            setPathState(5);
                        }
                        break;

                    case 5:
                        if (!follower.isBusy()) {
                            follower.followPath(line6GPP);
                            setPathState(6);
                        }
                        break;

                    case 6:
                        if (!follower.isBusy()) {
                            follower.followPath(line7GPP);
                            setPathState(7);
                        }
                        break;

                    case 7:
                        if (!follower.isBusy()) {
                            follower.followPath(line8GPP);
                            setPathState(8);
                        }
                        break;

                    case 8:
                        if (!follower.isBusy()) {
                            follower.followPath(line9GPP);
                            setPathState(9);
                        }
                        break;

                    case 9:
                        if (!follower.isBusy()) {
                            follower.followPath(line10GPP);
                            setPathState(10);
                        }
                        break;

                    case 10:
                        if (!follower.isBusy()) {
                            follower.followPath(line11GPP);
                            setPathState(11);
                        }
                        break;

                    case 11:
                        if (!follower.isBusy()) {
                            follower.followPath(line12GPP);
                            setPathState(12);
                        }
                        break;

                    case 12:
                        if (!follower.isBusy()) {
                            setPathState(-1);
                        }
                        break;
                }
                break;

            case 22:
                switch (pathState) {

                }
            }

    }

    private void transfer() {
        in.transfer();
        out.unblock();
        transfering = true;
    }

    private void stopTransfer() {
        in.stop();
        transfering = false;
    }

    private void intake() {
        out.block();
        in.runIntake(1.0);
        in.runTransfer(-0.5);
        intaking = true;
    }

    private void stopIntake() {
        in.stop();
        out.block();
        intaking = false;
    }


    private PathChain line1GPP, line2GPP, line3GPP, line4GPP, line5GPP, line6GPP,
            line7GPP, line8GPP, line9GPP, line10GPP, line11GPP, line12GPP;

    public void buildPaths() {
        line1GPP = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(40.000, 8.200),
                                new Pose(56.800, 19.200),
                                new Pose(62.000, 14.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(116))
                .build();

        line2GPP = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.000, 14.000),
                                new Pose(56.300, 27.200),
                                new Pose(41.000, 35.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(116), Math.toRadians(180))
                .build();

        line3GPP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(41.000, 35.500), new Pose(9.000, 35.500)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        line4GPP = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(26.000, 35.500),
                                new Pose(50.000, 48.000),
                                new Pose(53.500, 90.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();

        line5GPP = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.500, 90.000),
                                new Pose(18.000, 117.000),
                                new Pose(17.000, 96.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(280))
                .build();

        line6GPP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(17.000, 96.500), new Pose(16.600, 94.000)))
                .setLinearHeadingInterpolation(Math.toRadians(280), Math.toRadians(284))
                .build();

        line7GPP = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.600, 94.000),
                                new Pose(34.000, 108.000),
                                new Pose(43.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(284), Math.toRadians(180))
                .build();

        line8GPP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(43.000, 84.000), new Pose(31.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        line9GPP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(31.000, 84.000), new Pose(53.500, 90.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();

        line10GPP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(53.500, 90.000), new Pose(41.500, 59.400)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .build();

        line11GPP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(41.500, 59.400), new Pose(26.000, 59.400)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        line12GPP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(26.000, 59.400), new Pose(53.500, 90.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();
    }

    private PathChain line1PGP, line2PGP, line3PGP, line4PGP, line5PGP, line6PGP,
            line7PGP, line8PGP, line9PGP, line10PGP, line11PGP, line12PGP;


}
