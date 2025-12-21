package org.firstinspires.ftc.teamcode.auto.AutoPaths.BlueAutos;

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

@Config
@Configurable
@Autonomous(name="Blue Side Auto With Gate TEST TEST TEST",group="Blue Autos", preselectTeleOp="MainTeleOp")
public class BlueSideWithGateTESTTESTTEST extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    private int ballsToShoot;
    private int cycle = 0;
    private int CLOSE_SHOOTER_POWER = 3500;

    Timer shootTime = new Timer();
    private StaticShooter shooter;
    private IntakeSubsystem in;
    private OuttakeSubsystem out;
    private LimelightSubsystem limelight;

    private boolean intaking, transfering, scoring, moving;

    private boolean headingLockEnabled;
    private BeamBreakHelper intakeBeamBreak, outtakeBeamBreak;
    private Thread outtakeThread;
    private final Pose startPose = new Pose(33.5, 135.5, Math.toRadians(180));

    private PathChain line1, line2, line3, line4, line5, line6,
            line7, line8, line9, line10;
    public void buildPaths() {
        line1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(33.500, 135.500), new Pose(44.000, 100.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .build();

        line2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(44.000, 100.000),
                                new Pose(50.000, 70.000),
                                new Pose(8.000, 62.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(142))
                .build();

        line3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(11.000, 62.500), new Pose(44.000, 100.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(134))
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
        setPathState(-2);
    }

    @Override
    public void loop() {
        intakeBeamBreak.update();
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
            case -2:
                if (!follower.isBusy()) { //goes to shooting position
                    follower.setMaxPower(1);
                    shooter.setTargetRPM(CLOSE_SHOOTER_POWER);
                    out.aimClose();

                    setPathState(-1);
                }
                break;

            case -1:

                if (!follower.isBusy()) {
                    follower.followPath(line1);
                    shootTime.resetTimer();
                    setPathState(0);
                }
                break;

            case 0:
                if (!follower.isBusy()) {
                    if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot) && pathTimer.getElapsedTimeSeconds() < 9) {
                        if (shooter.getShooterVelocity() >= 3000) {
                            transfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                        outtakeBeamBreak.resetBallCount();
                        setPathState(1);
                    }
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    intake();
                    //follower.setMaxPower(0.7);
                    follower.followPath(line2);
                    ballsToShoot = 3;
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 3.8){
                    follower.followPath(line3);
                    //pathTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    if (!(outtakeBeamBreak.getBallCount() >= ballsToShoot) && pathTimer.getElapsedTimeSeconds() < 5) {
                        if (shooter.getShooterVelocity() >= 200) {
                            transfer();
                        }
                    } else {
                        stopTransfer();
                        out.block();
                        intake();
                        outtakeBeamBreak.resetBallCount();
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if (cycle < 3){
                    cycle++;
                    ballsToShoot = 3;
                    setPathState(1);
                }
                else{
                    out.aimScoring();
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