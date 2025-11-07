package org.firstinspires.ftc.teamcode.teleOp.competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BeamBreakHelper;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter;
import org.firstinspires.ftc.teamcode.subsystems.GobildaRGBIndicatorHelper;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.support.AlliancePresets;

import org.firstinspires.ftc.teamcode.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;

import java.util.Locale;

@Config
@TeleOp(group="1")
public class MainTeleOp extends CommandOpMode {
    // HARDWARE
    private StaticShooter shooter;
    private MecanumDrivebase drive;
    private OuttakeSubsystem out;
    private IntakeSubsystem in;
    private GoBildaPinpointDriver pinpoint;
    private LimelightSubsystem limelight;
    private GobildaRGBIndicatorHelper rgbHelper;
    private BeamBreakHelper beamBreak;
    private boolean aimServoLimit = true;
    private boolean calculateHood = true;

    // HEADING LOCK STUFF
    private boolean headingLockEnabled = false;
    public static double tP = 0.02;

    // CONTROLLERS
    private GamepadEx driver, manipulator;

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
        beamBreak = new BeamBreakHelper(hardwareMap);

        // Default Commands
        // Intake Command
        in.setDefaultCommand(new IntakeCommand(in, out, driver));

        // Shooting
        // Click bumper once to activate intake at full speed
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(()-> {
                    shooter.setTargetRPM(3200);
                    out.aimScoring();
                }));

        // Click bumper once to activate intake at close speed
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(()-> {
                    shooter.setTargetRPM(2500);
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
                        .whenPressed(new TransferCommand(in, out, driver));

        //Heading Lock
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                        .whenPressed(new InstantCommand(()-> headingLockEnabled = !headingLockEnabled));



        //Hood calculator
        manipulator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(()-> calculateHood = true));
        manipulator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(()-> calculateHood = false));


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
        beamBreak.update();

        out.aiming(gamepad1.cross, gamepad1.triangle);

        double forward = driver.getLeftY(); // Forwards/backwards
        double right = driver.getLeftX(); // Strafe
        double rotate = driver.getRightX(); // Rotation

//        if (headingLockEnabled && limelight.hasValidTarget()) {
//            LLResult result = limelight.getLatest();
//            if (result != null && result.isValid()) {
//                double finalRotation = result.getTxNC() * tP;
//                finalRotation = Math.max(-0.4, Math.min(finalRotation, 0.4));
//                rotate = finalRotation;
//            }
//        }

        if(calculateHood && limelight.hasValidTarget()){
            double hoodAngle = shooter.calculateHoodAngle(
                    limelight.getDistanceToTagCenterInches(true),
                    11.976,   // shooter height (in)
                    38.75,   // goal center height (in)
                    shooter.getShooterVelocity(),
                    false   // use low arc
            );

            out.aimServo(hoodAngle);
        }
        Pose2D pose;
        pose = driveFieldRelative(forward, right, rotate);

        if (shooter.isAtTargetThreshold()) {
            rgbHelper.setColour(GobildaRGBIndicatorHelper.Colour.BLUE);
        } else if (beamBreak.isBeamStable()) {
            rgbHelper.setColour(GobildaRGBIndicatorHelper.Colour.GREEN);
        } else {
            rgbHelper.setColour(GobildaRGBIndicatorHelper.Colour.RED);
        }

        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES)
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(()-> {
                    pinpoint.resetPosAndIMU();
                }));

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
        telemetry.addData("Hood on?", calculateHood);
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
        telemetry.addData("Beam Break State: ", beamBreak.getBeamState());
        telemetry.addData("RGB Colour", rgbHelper.getCurrentColour());
        telemetry.addLine();
        telemetry.addLine("----  Pinpoint Data  ----");
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.addData("Soft limit On?", aimServoLimit);
        telemetry.update();
    }

    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();  // Current position

        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
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
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }
}
