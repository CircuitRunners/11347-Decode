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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BeamBreakHelper;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;

import java.util.List;

@Config
@Configurable
@Autonomous(name="Blue Side Auto",group="Blue Autos", preselectTeleOp="MainTeleOp")
public class BlueSideAutoFar extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    Timer shootTime = new Timer();
    private StaticShooter shooter;
    private IntakeSubsystem in;
    private OuttakeSubsystem out;
    private LimelightSubsystem limelight;

    private boolean intaking, transfering, scoring, moving;

    private boolean headingLockEnabled;

    private final Pose startPose = new Pose(40.0, 8.2, Math.toRadians(180));
    private PathChain line1, line2, line3, line4, line5, line6,
            line7, line8;
    public void buildPaths() {
        line1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(40.000, 8.200),
                                new Pose(56.000, 19.200),
                                new Pose(55.00, 14.00)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();

        line2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.800, 19.200),
                                new Pose(56.300, 27.200),
                                new Pose(41.000, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        line3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.000, 35.000), new Pose(8.000, 35.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        line4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(8.000, 35.000),
                                new Pose(45.000, 35.000),
                                new Pose(55.000, 14.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();

        line5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.000, 14.000),
                                new Pose(35.000, 29.000),
                                new Pose(8.000, 28.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(270))
                .build();

        line6 = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(8.000, 28.000), new Pose(8.000, 9.000)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        line7 = follower
                .pathBuilder()

                .addPath(
                        new BezierCurve(
                                new Pose(8.000, 9.000),
                                new Pose(35.000, 18.000),
                                new Pose(55.000, 14.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(114))
                .build();
        line8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(55.000, 14.000), new Pose(35.000, 9.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(0))
                .build();
    }


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
        telemetry.addData("Team ID:", AlliancePresets.getAllianceShooterTag());
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
        switch (pathState) {
            case -2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    shooter.setTargetRPM(3400);
                    out.aimScoring();
                    setPathState(0);
                }
                break;

            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(line1);
                    shootTime.resetTimer();
                    setPathState(-1);
                }
                break;

            case -1:
                if (shootTime.getElapsedTimeSeconds() < 8) {
                    if (shooter.isAtTargetThreshold()) {
                        transfer();
                    } else if (shooter.getShooterVelocity() < 3100) {
                        stopTransfer();
                    }
                } else {
                    stopTransfer();
                    out.block();
                    intake();
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    intake();
                    follower.setMaxPower(1);
                    follower.followPath(line2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(line3);
                    setPathState(3);
                }
                break;


            case 3:
                if (!follower.isBusy()) {
                    //stopIntake();
                    shooter.setTargetRPM(3400);
                    follower.followPath(line4);
                    setPathState(-4);
                }
                break;

            case -4:
                if (!follower.isBusy()) {
                    if (shootTime.getElapsedTimeSeconds() < 18) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        } else if (shooter.getShooterVelocity() < 3100) {
                            stopTransfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    intake();
                    follower.followPath(line5);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(line6);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    //stopIntake();
                    follower.setMaxPower(1);
                    shooter.setTargetRPM(3400);
                    follower.followPath(line7);
                    setPathState(-9);
                }
                break;

//            case 7:
//                if (!follower.isBusy()) {
//                    follower.followPath(line8);
//                    setPathState(8);
//                }
//                break;

//            case 8:
//                if (!follower.isBusy()) {
//                    setPathState(-9);
//                }
//                break;

            case -9:
                if (!follower.isBusy()) {
                    if (shootTime.getElapsedTimeSeconds() < 28.5) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        } else if (shooter.getShooterVelocity() < 3100) {
                            stopTransfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                        setPathState(9);
                    }
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    shooter.eStop();

                    follower.followPath(line8);
                    //setPathState(10);
                }
                break;
//
//            case 10:
//                if (!follower.isBusy()) {
//                    shooter.eStop();
//                    follower.followPath(line11);
//                    setPathState(11);
//                    //follower.pausePathFollowing();
//
//                }
//                break;
//
//            case 11:
//                if (!follower.isBusy()) {
//                    stopIntake();
//                    shooter.setTargetRPM(2550);
//                    follower.followPath(line12);
//                    setPathState(12);
//                }
//                break;
//
//            case 12:
//                if (!follower.isBusy()) {
//                    if (shootTime.getElapsedTimeSeconds() < 28) {
//                        if (shooter.isAtTargetThreshold()) {
//                            transfer();
//                        } else if (shooter.getShooterVelocity() < 2300) {
//                            stopTransfer();
//                        }
//                    } else {
//                        stopTransfer();
//                        out.block();
//                        shooter.setTargetRPM(0);
//                        setPathState(13);
//                    }
//                }
//                break;
//            case 13:
//                if (!follower.isBusy()) {
//                    shooter.eStop();
//                    follower.followPath(line13);
//                }
//        }
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
}