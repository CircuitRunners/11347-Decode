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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;

import java.util.List;
//@Disabled
@Config
@Configurable
@Autonomous(name="Red Side Far 9 ball auto", group="Red Autos", preselectTeleOp="MainTeleOp")
public class RedSideAlt2 extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    private int ballsToShoot;

    Timer shootTime = new Timer();
    private StaticShooter shooter;
    private IntakeSubsystem in;
    private OuttakeSubsystem out;
    private LimelightSubsystem limelight;
    private BeamBreakHelper intakeBeamBreak, outtakeBeamBreak;
    private Thread outtakeThread;

    private boolean intaking, transfering, scoring, moving;
    private boolean headingLockEnabled;

    private final Pose startPose = new Pose(104.0, 8.2, Math.toRadians(0));
    private PathChain line1, line2, line3, line4, line5, line6,
            line7, line8, line9, line10, line11, line12;

    public void buildPaths() {
        line1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(104.000, 8.200),
                        new Pose(110.200, 22.200),
                        new Pose(90.000, 14.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66.5))
                .build();

        line2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(89.000, 14.000),
                        new Pose(87.700, 27.200),
                        new Pose(103.000, 35.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(66.5), Math.toRadians(0))
                .build();

        line3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(103.000, 35.000),
                        new Pose(134.500, 35.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(134.500, 35.000),
                        new Pose(80.000, 48.000),
                        new Pose(89, 14)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66.5))
                .build();

        line5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(89.000, 14.000),
                        new Pose(87.700, 27.200),
                        new Pose(103.000, 59.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(66.5), Math.toRadians(0))
                .build();

        line6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(103.000, 59.000),
                        new Pose(134.500, 59.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(134.500, 59.000),
                        new Pose(80.000, 48.000),
                        new Pose(89.000, 14.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66.5))
                .build();

        line10 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(89.000, 14.000),
                        new Pose(87.700, 27.200),
                        new Pose(103.000, 35.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(66.5), Math.toRadians(180))
                .build();
//
//        line11 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Pose(102.500, 58.000),
//                        new Pose(132.000, 58.000)
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .build();
//
//        line12 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Pose(132.000, 58.500),
//                        new Pose(102.000, 69.000),
//                        new Pose(90.500, 90.000)
//                ))
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                .build();
    }


    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);
        ballsToShoot = 3;
        out = new OuttakeSubsystem(hardwareMap);
        in = new IntakeSubsystem(hardwareMap);
        intakeBeamBreak = new BeamBreakHelper(hardwareMap, "intakeBeamBreak", 3);
        outtakeBeamBreak = new BeamBreakHelper(hardwareMap, "outtakeBeamBreak", 0);
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
        outtakeThread = new Thread(() -> {
            while (!Thread.interrupted()) {
                outtakeBeamBreak.update();
                try {
                    Thread.sleep(1);
                } catch (Exception e) {
                    break;
                }
            }
        });

        outtakeThread.start();

        pathTimer.resetTimer();
        setPathState(-2);
    }

    @Override
    public void loop() {
        intakeBeamBreak.update();
        follower.update();
        shooter.update();
        limelight.update();
        autonomousPathUpdate();

        if (shootTime.getElapsedTimeSeconds() > 28) {
            setPathState(9);
        }

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
        telemetry.addData("Balls Shot", outtakeBeamBreak.getBallCount());
        telemetry.addData("Ball Held", intakeBeamBreak.getBallCount() % 3);
        telemetry.addData("Balls to Shoot", ballsToShoot);
        telemetry.addData("Outtake Raw", outtakeBeamBreak.isBeamBroken());
        telemetry.addData("Timer: ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading (deg)", "%.1f", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();

        if (outtakeThread != null) {
            outtakeThread.interrupt();
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            //Sets up before movement
            case -2:
                if (!follower.isBusy()) {
                    shooter.setTargetRPM(3400);
                    out.aimScoring();
                    setPathState(0);
                }
                break;

            //Starts movement
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(line1);
                    shootTime.resetTimer();
                    setPathState(-1);
                }
                break;

            //Shoots balls until intake is empty and then resets intake count
            case -1:
                if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot)) {
                    if (shooter.isAtTargetThreshold()) {
                        transfer();
                    } else if (shooter.getShooterVelocity() < 3250) {
                        stopTransfer();
                    }
                } else {
                    stopTransfer();
                    out.block();
                    out.aimScoring();
                    intake();
                    intakeBeamBreak.resetBallCount();
                    setPathState(1);
                }
                break;

            //Makes sure intake has started, resets outtake count (in this case so that previous
            // case doesn't get errors), and moves
            case 1:
                if (!follower.isBusy()) {
                    intake();
                    outtakeBeamBreak.resetBallCount();
                    follower.followPath(line2);
                    setPathState(2);
                }
                break;

            //Moves to collect balls
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(line3);
                    setPathState(3);
                }
                break;

            //Moves to next score position and changes target RPM
            case 3:
                if (!follower.isBusy()) {
                    stopIntake();
                    shooter.setTargetRPM(3400);
                    follower.followPath(line4);
                    setPathState(-4);
                }
                break;

            case -4:
                if (!intakeBeamBreak.isBeamStable()) {
                    ballsToShoot = 3;
                } else {
//                    if (2 >= intakeBeamBreak.getBallCount()){
//                        ballsToShoot = intakeBeamBreak.getBallCount();
//                    }
                    ballsToShoot = Range.clip(intakeBeamBreak.getBallCount(), 0, 2);
                }
                if (!follower.isBusy()) {
                    if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot)) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        } else if (shooter.getShooterVelocity() < 3200) {
                            stopTransfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                        intakeBeamBreak.resetBallCount();
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    intake();
                    outtakeBeamBreak.resetBallCount();
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
                    stopIntake();
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
                if (!intakeBeamBreak.isBeamStable()) {
                    ballsToShoot = 3;
                } else {
//                    if (2 >= intakeBeamBreak.getBallCount()){
//                        ballsToShoot = intakeBeamBreak.getBallCount();
//                    }
                    ballsToShoot = Range.clip(intakeBeamBreak.getBallCount(), 0, 2);
                }
                if (!follower.isBusy()) {
                    if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot)) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        } else if (shooter.getShooterVelocity() < 3200) {
                            stopTransfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        shooter.eStop();
//                        intake();
                        setPathState(9);
                    }
                }
                break;

            case 9:
                if (!follower.isBusy()) {
//                    intake();
                    outtakeBeamBreak.resetBallCount();
                    follower.followPath(line10);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.pausePathFollowing();
//                    follower.followPath(line11);
//                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    stopIntake();
                    shooter.setTargetRPM(2400);
                    follower.followPath(line12);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    if (shootTime.getElapsedTimeSeconds() < 31) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        } else if (shooter.getShooterVelocity() < 2250) {
                            stopTransfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                        shooter.setTargetRPM(0);
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
