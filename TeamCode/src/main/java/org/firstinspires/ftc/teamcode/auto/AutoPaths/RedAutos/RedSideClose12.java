package org.firstinspires.ftc.teamcode.auto.AutoPaths.RedAutos;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BeamBreakHelper;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;

import java.util.List;
@Disabled
@Config
@Configurable
@Autonomous(name="Red Side Auto Close 12",group="Red Autos", preselectTeleOp="MainTeleOp")
public class RedSideClose12 extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    private int ballsToShoot;

    private int CLOSE_SHOOTER_POWER = 3700;

    Timer shootTime = new Timer();
    private StaticShooter shooter;
    private IntakeSubsystem in;
    private OuttakeSubsystem out;
    private LimelightSubsystem limelight;

    private boolean intaking, transfering, scoring, moving;

    private boolean headingLockEnabled;
    private BeamBreakHelper intakeBeamBreak, outtakeBeamBreak;
    private Thread outtakeThread;
    private final Pose startPose = new Pose(110.0, 135.5, Math.toRadians(0));

    private PathChain line1, line2, line3, line4, line5, line6,
            line7, line8, line9, line10, line11, line12, line13;
    public void buildPaths() {

        line1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(110.00, 135.500), new Pose(100.000, 100.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))
                .build();

        line2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 100.000), new Pose(97.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                .build();

        line3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.000, 84.000), new Pose(125.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(125.000, 84.000),
                                new Pose(120.000, 78.000),//80
                                new Pose(119.500, 77.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(119.500, 77.000), new Pose(125.000, 77.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(129.000, 77.000), new Pose(100.000, 100.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))
                .build();

        line7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 100.000), new Pose(97.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                .build();

        line8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.000, 60.000), new Pose(133.500, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line9 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133.500, 60.000),
                                new Pose(98.000, 64.000),//80
                                new Pose(100.000, 100.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))
                .build();



        line10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 100.000), new Pose(97.000, 35.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                .build();

        line11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(97.000, 35.000), new Pose(133.500, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        line12 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(133.500, 36.000), new Pose(100.000, 100.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))
                .build();
        line13 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 100.000), new Pose(100.000, 75.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                .build();


    }


    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        ballsToShoot = 3;
        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);
        out = new OuttakeSubsystem(hardwareMap);
        in = new IntakeSubsystem(hardwareMap);
        intakeBeamBreak = new BeamBreakHelper(hardwareMap, "intakeBeamBreak", 3);
        outtakeBeamBreak = new BeamBreakHelper(hardwareMap, "outtakeBeamBreak", 0);
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
        setPathState(0);
    }

    @Override
    public void loop() {
        intakeBeamBreak.update();
        follower.update();
        shooter.update();
        limelight.update();
        autonomousPathUpdate();

        telemetry.addLine("----  RED Side Auto Close 12  ----");
        telemetry.addLine();
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
            case 0:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.87);
                    shooter.setTargetRPM(CLOSE_SHOOTER_POWER);
                    out.aimClose();
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(line1);
                    shootTime.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot) && pathTimer.getElapsedTimeSeconds() < 3.5 && pathTimer.getElapsedTimeSeconds() > 0.8) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                        outtakeBeamBreak.resetBallCount();
                        setPathState(3);
                    }
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(line2);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(line3);
                    ballsToShoot = 3;
                    setPathState(5);
                }
                break;


            case 5:
                if (!follower.isBusy()) {
                    shooter.setTargetRPM(CLOSE_SHOOTER_POWER);
                    follower.setMaxPower(0.67);
                    stopIntake();
                    follower.followPath(line4);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(line5);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.87);
                    follower.followPath(line6);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot) && pathTimer.getElapsedTimeSeconds() < 3 && pathTimer.getElapsedTimeSeconds() > 0.8) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                        outtakeBeamBreak.resetBallCount();
                        setPathState(9);
                    }
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    ballsToShoot = 3;
                    follower.followPath(line7);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(line8);
                    setPathState(11);
                }
                break;

            case 11:
                shooter.setTargetRPM(CLOSE_SHOOTER_POWER);
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(line9);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot) && pathTimer.getElapsedTimeSeconds() < 3 && pathTimer.getElapsedTimeSeconds() > 0.8) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                        outtakeBeamBreak.resetBallCount();
                        setPathState(13);
                    }
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    ballsToShoot = 3;
                    follower.followPath(line10);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(line11);
                    setPathState(15);
                }
                break;

            case 15:

                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(line12);

                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot) && pathTimer.getElapsedTimeSeconds() < 3.5 && pathTimer.getElapsedTimeSeconds() > 0.8) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        shooter.eStop();
                        setPathState(17);
                    }
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(line13);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    shooter.eStop();
                    follower.pausePathFollowing();
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