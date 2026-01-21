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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name="Red Side Auto Far",group="Red Autos", preselectTeleOp="MainTeleOp")
public class RedSideAutoFar9 extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    private int ballsToShoot;
    private int FAR_SHOOTER_POWER = 4600;
    private int LOWEST_POWER = 4300;
    private double SHOOTING_ANGLE = 67;
    Timer shootTime = new Timer();
    private StaticShooter shooter;
    private IntakeSubsystem in;
    private OuttakeSubsystem out;
    private LimelightSubsystem limelight;

    private boolean intaking, transfering, scoring, moving;

    private boolean headingLockEnabled;
    private BeamBreakHelper intakeBeamBreak, outtakeBeamBreak;
    private Thread outtakeThread;
    private final Pose startPose = new Pose(104.0, 8.2, Math.toRadians(0));
    private PathChain line1, line2, line3, line4, line5, line6,
            line7, line8, line9, line10;
    public void buildPaths() {
        line1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(104.000, 8.200),
                                new Pose(88.000, 19.200),
                                new Pose(89.000, 14.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(SHOOTING_ANGLE)
                )
                .build();

        line2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(89.00, 14.00),
                                new Pose(87.700, 27.200),
                                new Pose(103.000, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(SHOOTING_ANGLE),
                        Math.toRadians(0)
                )
                .build();

        line3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(103.000, 35.000),
                                new Pose(136.000, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();

        line4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(136.000, 35.000),
                                new Pose(99.000, 35.000),
                                new Pose(89.000, 14.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(SHOOTING_ANGLE)
                )
                .build();

        line5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(89.000, 14.000),
                                new Pose(109.000, 20.000),
                                new Pose(119.000, 14.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(SHOOTING_ANGLE),
                        Math.toRadians(0.0)
                )
                .build();

        line6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(119.000, 14.000),
                                new Pose(135.000, 14.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0.0),
                        Math.toRadians(0.0)
                )
                .build();

        line7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.000, 14.000),
                                new Pose(119.000, 9.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0.0),
                        Math.toRadians(0.0)
                )
                .build();

        line8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(119.000, 9.000),
                                new Pose(135.000, 9.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        line9 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(135.000, 9.000),
                                new Pose(124.000, 30.000),
                                new Pose(89.000, 14.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(0.0),
                        Math.toRadians(SHOOTING_ANGLE)
                )
                .build();

        line10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(89.000, 14.000),
                                new Pose(109.000, 15.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(SHOOTING_ANGLE),
                        Math.toRadians(0.0)
                )
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

        telemetry.addLine("----  RED Side Auto Far 9 (Loading Zone)  ----");
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
                    follower.setMaxPower(1);
                    shooter.setTargetRPM(FAR_SHOOTER_POWER);
                    out.aimScoring();
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
                    if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot) && pathTimer.getElapsedTimeSeconds() < 6) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        } else if (shooter.getShooterVelocity() < LOWEST_POWER) {
                            stopTransfer();
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
                    ballsToShoot = 3;
                    follower.followPath(line3);
                    setPathState(5);
                }
                break;


            case 5:
                if (!follower.isBusy()) {
                    stopIntake();
                    shooter.setTargetRPM(FAR_SHOOTER_POWER);
                    follower.setMaxPower(1);
                    follower.followPath(line4);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot) && (pathTimer.getElapsedTimeSeconds() < 7)) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        } else if (shooter.getShooterVelocity() < LOWEST_POWER) {
                            stopTransfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                        outtakeBeamBreak.resetBallCount();
                        setPathState(7);
                    }
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(line5);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(line6);
                    setPathState(9);
                }

                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8); //0.8
                    shooter.setTargetRPM(FAR_SHOOTER_POWER);
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
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(line9);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot) && pathTimer.getElapsedTimeSeconds() < 7.5 && shootTime.getElapsedTimeSeconds() < 28.5) {
                        if (shooter.isAtTargetThreshold()) {
                            transfer();
                        } else if (shooter.getShooterVelocity() < LOWEST_POWER) {
                            stopTransfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        outtakeBeamBreak.resetBallCount();
                        setPathState(13);
                    }
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    shooter.eStop();

                    follower.followPath(line10);

                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    shooter.eStop();
                    stopIntake();
                    follower.pausePathFollowing();
                }
                break;
        }
    }

    private void transfer() {
        in.transfer(0.75);
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