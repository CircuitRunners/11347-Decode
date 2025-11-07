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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;

import java.util.List;

@Disabled
@Config
@Configurable
@Autonomous(name="Blue Side Close Auto",group="Blue Autos", preselectTeleOp="MainTeleOp")
public class BlueSideClose extends OpMode {
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

    private final Pose startPose = new Pose(17.8, 121, Math.toRadians(234));
    private PathChain line1, line2, line3, line4, line5, line6,
            line7, line8, line9, line10, line11, line12;

    public void buildPaths() {
        line1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(17.800, 121.000), new Pose(53.500, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(234), Math.toRadians(135))
                .build();

        line2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(53.500, 90.000), new Pose(42.000, 83.500)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        line3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42.000, 83.500), new Pose(16.000, 83.500)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        line4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(16.000, 83.500), new Pose(53.500, 90.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        line5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(53.500, 90.000), new Pose(41.500, 59.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        line6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(41.500, 59.000), new Pose(9.000, 59.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        line7 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.000, 59.000),
                                new Pose(42.000, 69.000),
                                new Pose(53.500, 90.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        line8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(53.500, 90.000), new Pose(41.000, 35.500)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        line9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(41.000, 35.500), new Pose(11.000, 35.500)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        line10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(11.000, 35.500), new Pose(53.500, 90.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
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
        AlliancePresets.setAllianceShooterTag(AlliancePresets.Alliance.RED.getTagId());
        limelight.setAllianceTagID(AlliancePresets.getAllianceShooterTag());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        follower.update();

        buildPaths();
        pathTimer = new Timer();
        pathTimer.resetTimer();

        AlliancePresets.setAllianceShooterTag(AlliancePresets.Alliance.RED.getTagId());

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Team ID:", AlliancePresets.getAllianceShooterTag());
        telemetry.update();
    }

    @Override
    public void init_loop() {
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
                    shooter.setTargetRPM(2450);
                    out.aimClose();
                    setPathState(0);
                }
                break;

            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(line1);
                    shootTime.resetTimer();
                    setPathState(-1);
                }

            case -1:
                if (!follower.isBusy()) {
                    if (shootTime.getElapsedTimeSeconds() < 5) {
                        if (shooter.getShooterVelocity() < 2250) {
                            out.setAim(0.16);
                        } else {
                            out.aimClose();
                        }
                        transfer();
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                        setPathState(1);
                    }
                }

                break;

            case 1:
                if (!follower.isBusy()) {
                    intake();
                    follower.followPath(line2);
                    setPathState(2);
                }

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(line3);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    stopIntake();
                    shooter.setTargetRPM(2450);
                    follower.followPath(line4);
                    setPathState(-4);
                }
                break;

            case -4:
                if (!follower.isBusy()) {
                    if (shootTime.getElapsedTimeSeconds() < 12) {
                        if (shooter.getShooterVelocity() < 2250) {
                            out.setAim(0.16);
                        } else {
                            out.aimClose();
                        }
                        transfer();
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
                    follower.followPath(line6);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    shooter.setTargetRPM(2450);
                    follower.followPath(line7);
                    setPathState(-9);
                }
                break;

            case -9:
                stopIntake();
                if (!follower.isBusy()) {
                    if (shootTime.getElapsedTimeSeconds() < 22) {
                        if (shooter.getShooterVelocity() < 2250) {
                            out.setAim(0.16);
                        } else {
                            out.aimClose();
                        }
                        transfer();
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
                    intake();
                    follower.followPath(line8);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(line9);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    stopIntake();
                    shooter.setTargetRPM(2450);
                    follower.followPath(line10);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    if (shootTime.getElapsedTimeSeconds() < 31) {
                        if (shooter.getShooterVelocity() < 2250) {
                            out.setAim(0.16);
                        } else {
                            out.aimClose();
                        }
                        transfer();
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                    }
                }
                break;
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
