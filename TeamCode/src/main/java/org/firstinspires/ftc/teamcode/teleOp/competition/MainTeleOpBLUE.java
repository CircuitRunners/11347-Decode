package org.firstinspires.ftc.teamcode.teleOp.competition;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BeamBreakHelper;
import org.firstinspires.ftc.teamcode.subsystems.GobildaRGBIndicatorHelper;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;

import java.util.Locale;

@Config
@TeleOp(group="1")
public class MainTeleOpBLUE extends CommandOpMode {
    // HARDWARE
    private StaticShooter shooter;
    private MecanumDrivebase drive;
    private OuttakeSubsystem out;
    private IntakeSubsystem in;
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private LimelightSubsystem limelight;
    private GobildaRGBIndicatorHelper rgbHelper;
    private BeamBreakHelper intakeBeamBreak;
    private boolean aimServoLimit = true;

    // HEADING LOCK STUFF
    private boolean headingLockEnabled = false;
    public static double tP = 0.02;
    private double METERS_TO_INCH = 39.37;

    // CONTROLLERS
    private GamepadEx driver, manipulator;
    //VELOCITY AND HOOD CALCULATIONS (TUNED)

    private Pose GOAL_POS_BLUE = new Pose(138, 6);
    private double SCORE_HEIGHT = 25;
    private double SCORE_ANGLE = Math.toRadians(-30);
    private double PASS_THROUGH_POINT_RADIUS =5;
    private double HOOD_MAX_ANGLE = Math.toRadians(67);
    private double HOOD_MIN_ANGLE = Math.toRadians(0);
    private double wheelDiameter = 3.21;
    private double maxHoodTicks = 0.5;
    private double hoodAngle = 0;
    private double flywheelSpeed = 0;
    public static boolean usePhysics = false;

    //AUTO TURN STUFF
    public static boolean runAutoTurn = false;
    public static double headingSetPoint = 113.5;
    public static double autoTurnKP = 0.9;
    public static double maxTurnVelocity = 0.7; //need to use this more/tune this so i can get more agressive P
    private double error = 0;
    public static double autoTurnErrorMax = 11;
    public static double autoTurnKD = 0.38;
    public static double goalOffset = 0;
    private double lastError = 0.0;
    private double lastTime = 0.0;   // seconds

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));

        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);
        drive = new MecanumDrivebase(hardwareMap);
        out = new OuttakeSubsystem(hardwareMap);
        in = new IntakeSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, "limelight");
        limelight.setAllianceTagID(AlliancePresets.getAllianceShooterTag());
        rgbHelper = new GobildaRGBIndicatorHelper(hardwareMap);
        intakeBeamBreak = new BeamBreakHelper(hardwareMap, "intakeBeamBreak", 0);
        follower = Constants.createFollower(hardwareMap);

        // Default Commands
        // Intake Command
        in.setDefaultCommand(new IntakeCommand(in, out, driver));

        // Shooting
        // Click bumper once to activate intake at full speed
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(()-> {
                    shooter.setTargetRPM(4600);
                    out.aimScoring();
                }));

        // Click bumper once to activate intake at close speed
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(()-> {
                    shooter.setTargetRPM(3700);
                    out.aimClose();
                }));

        manipulator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(()-> {
                    aimServoLimit = false;
                }));

        manipulator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(()-> {
                    aimServoLimit = true;
                }));

        // Click both bumpers to turn shooter off
        Trigger shooterOff = driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER));
        shooterOff.whenActive(new InstantCommand(() -> shooter.setTargetRPM(0)));

        // Transfering Command
        // Click to toggle on and off transfering
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new TransferCommand(in, out, driver, shooter));


        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(()-> {

        follower.setPose(new Pose(72,72, Math.toRadians(0)));

                }));
        telemetry.addLine("Pinpoint Reset - Position now 72,72 (Middle)!");

        //Heading Lock
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(()-> headingLockEnabled = !headingLockEnabled));

        telemetry.addLine("ROBOT READY!");
        telemetry.addData("Team ID:", AlliancePresets.getAllianceShooterTag());
        telemetry.addData("Current Alliance Tag", limelight.getLimelightAllianceTagID());
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        shooter.update();
        limelight.update();
        intakeBeamBreak.update();
        follower.update();

        out.aiming(gamepad1.cross, gamepad1.triangle);

        Pose2D p = pinpoint.getPosition();
        double x = p.getX(DistanceUnit.INCH);
        double y = p.getY(DistanceUnit.INCH);
        double heading = p.getHeading(AngleUnit.RADIANS);

        calculateHoodPos(x, y, heading, follower.getVelocity());


        double forward = driver.getLeftY(); // Forwards/backwards
        double right = driver.getLeftX(); // Strafe
        double rotate = driver.getRightX(); // Rotation

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(()-> {
                    //pinpoint.resetPosAndIMU();
                    Pose2D p2 = pinpoint.getPosition();
                    double x2 = p2.getX(DistanceUnit.INCH);
                    double y2 = p2.getY(DistanceUnit.INCH);
                    pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x2,y2, AngleUnit.RADIANS, 0.0));
                }));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(()-> {
                    runAutoTurn = true;
                }))
                .whenReleased(new InstantCommand(()-> {
                    runAutoTurn = false;
                }));


//        if (headingLockEnabled && limelight.hasValidTarget()) {
//            LLResult result = limelight.getLatest();
//            if (result != null && result.isValid()) {
//                double finalRotation = result.getTxNC() * tP;
//                finalRotation = Math.max(-0.4, Math.min(finalRotation, 0.4));
//                rotate = finalRotation;
//            }
//        }


        //auto turn stuff
        if(runAutoTurn){
            //yes i used gpt bro i dont know this stuff
            double currentTime = System.nanoTime() * 1e-9; // seconds
            double dt = currentTime - lastTime; //new
            headingSetPoint = getRobotPositionAngle(x, y);

            error = headingSetPoint - Math.toDegrees(heading);
            if (Math.abs(error-180) <autoTurnErrorMax){
                error = 0;
            }
            else {

                if (error > 180) {
                    error -= 360;
                } else if (error < -180) {
                    error += 360;
                }
            }

            double derivative = 0;
            if (dt > 0) {
                derivative = (error - lastError) / dt;
            } //new

            rotate = (autoTurnKP * error) + (autoTurnKD * derivative); //new
            rotate = Math.min(Math.max(rotate, -maxTurnVelocity), maxTurnVelocity);

            lastError = error;
            lastTime = currentTime; //new
        }

        Pose2D pose;
        pose = driveFieldRelative(forward, right, rotate);

        if (intakeBeamBreak.isBeamStable()) {
            rgbHelper.setColour(GobildaRGBIndicatorHelper.Colour.GREEN);
        } else {
            rgbHelper.setColour(GobildaRGBIndicatorHelper.Colour.RED);
        }




        //Velocity and hood stuff

        double gearRatio = 40.0 / 52.0;

        double wheelRPM = (flywheelSpeed * 60.0) / (Math.PI * (wheelDiameter/4.0));
        double motorRPM = wheelRPM * gearRatio;

        double hoodPos = (maxHoodTicks - Range.scale(hoodAngle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE, 0.0, 0.45));

        if (usePhysics) {
            shooter.setTargetRPM(motorRPM);
            out.setAim(hoodPos);
        }


        //data
        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES)
        );

        String followerData = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())

        );



        if (aimServoLimit) {
            if (out.getAimPos() > 0.55) {
                out.setAim(0.55);
            }
            if (out.getAimPos() < 0) {
                out.setAim(0);
            }
        }

        double distLOS = limelight.getDistanceToTagCenterInches(false);
        double distGround = limelight.getDistanceToTagCenterInches(true);


        // --- Telemetry ---
        // Limelight Distance Calc
        telemetry.addLine("----  Limelight Data  ----");
        telemetry.addData("Distance (LOS, in)", distLOS);
        telemetry.addData("Distance (Ground, in)", distGround);
        telemetry.addData("Tx", limelight.getTx());
        telemetry.addData("Ty", limelight.getTy());
        telemetry.addLine();
        telemetry.addLine("----  Subsystems Data  ----");
        telemetry.addData("Heading Lock Active for Team ID "+ AlliancePresets.getAllianceShooterTag() +"?", headingLockEnabled);
        telemetry.addData("Shooter Encoder Velo", shooter.getShooterVelocity());
        telemetry.addData("Aiming Servo Pos: ", out.getAimPos());
        telemetry.addData("Beam Break State: ", intakeBeamBreak.isBeamBroken());
        telemetry.addData("RGB Colour", rgbHelper.getCurrentColour());
        telemetry.addLine();
        telemetry.addLine("----  Pinpoint Data  ----");
        telemetry.addData("Position", data);
        telemetry.addData("Follower Position:", followerData);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.addData("Soft limit On?", aimServoLimit);
        telemetry.addData("Hood pos", hoodPos);
        telemetry.addData("Shooter Predicted Vel",motorRPM);
        telemetry.addData("Running auto lock", runAutoTurn);
        telemetry.addData("Heading variable (temp)", Math.toDegrees(heading));
        telemetry.addData("error?)", error);
        telemetry.update();
    }

    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        pinpoint.update();

        Pose2D pos = pinpoint.getPosition();  // Current position
//
        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit
                .normalizeRadians(theta - robotAngle);

        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        drive.drive(newForward, newRight, rotate);
        return pos;
    }

    private void configurePinpoint() {
        pinpoint.resetPosAndIMU();

        pinpoint.setOffsets(-28.042, -147.012, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
    }

    public void updateCoordinatesWithAprilTag() {
        limelight.limelight.updateRobotOrientation(follower.getHeading());
        limelight.limelight.pipelineSwitch(0);
        LLResult result = limelight.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D mt1Pose = result.getBotpose();
            if (mt1Pose != null) {
                double finalX = (mt1Pose.getPosition().y * METERS_TO_INCH) + 72.0;
                double finalY = (-mt1Pose.getPosition().x * METERS_TO_INCH) + 72.0;
                follower.setPose(new Pose(finalX, finalY, follower.getHeading()));
                gamepad1.rumble(500);
            }
        }
    }

    public void calculateHoodPos(double robotX, double robotY, double robotHeading, Vector robotVelocity) {
        // Horizontal distance to goal
        double dx = GOAL_POS_BLUE.getX() - robotX;
        double dy = GOAL_POS_BLUE.getY() - robotY;
        double distanceToGoal = Math.hypot(dx, dy);
        double angleToGoal = Math.atan(dy / dx);
        Vector robotToGoalVector = new Vector(distanceToGoal, angleToGoal);

        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

        //calculuate initial launch components
        hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);

        flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));
//
//        //get robot velocity and conver it into parallel and perpendicular components
//        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();
//
//        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
//        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();
//
//        //velocity compensation variables
//        double vz = 1flywheelSpeed * Math.sin(hoodAngle);
//        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
//        double ivr = x / time + parallelComponent;
//        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
//        double ndr = nvr * time;
//
//        //recalculuate launch components
//        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
//
//        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        //update robot position


    }
    public double getRobotPositionAngle(double robotX, double robotY){
        //This function is special and should not be touched
        double dxx = GOAL_POS_BLUE.getX() - robotX + goalOffset;
        double dyy = GOAL_POS_BLUE.getY() - robotY;
        double angleToGoal = Math.atan(dyy / dxx);
        return (Math.toDegrees(angleToGoal) + 180);
    }



}
