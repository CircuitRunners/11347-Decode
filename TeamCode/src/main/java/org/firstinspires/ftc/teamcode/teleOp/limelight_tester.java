package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.mecanumDB;

@Config
@TeleOp(name="limelight tester", group=".")
public class limelight_tester extends CommandOpMode {

    private mecanumDB drive;
    private int outtakePosition = -1;
    private int intakePosition = -1;
    GamepadEx driver;
    private boolean testVariable = false;
    private double launcherMotor = 0;
    private double feederServo = 0;
    private double intakeMotor = 0;
    double leftStickYVal;
    double rightStickXVal;
    private Timer pathTimer;
    private Limelight3A limelight;
    public static double HEADING_KP_TX = 0.023;
    public static double ROTATION_MIN_POWER = 0.0;
    private boolean isHeadingLocked = false;
    double finalRotation;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);

        drive = new mecanumDB();
        pathTimer = new Timer();
        drive.init(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();


//        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(new InstantCommand(() -> {
//                    outtakePosition = (outtakePosition + 1) % 2;
//                    testVariable = true;
//                    pathTimer.resetTimer();
//                    //if needed test intake using left and right bumper?
//                }));
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(new InstantCommand(() -> {
//                    intakePosition = (intakePosition + 1) % 2;
//
//                }));
////        driver.getGamepadButton(GamepadKeys.Button.X)
////                .whenPressed(new InstantCommand(() -> {
////
////                }));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    isHeadingLocked = true;
                }));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    isHeadingLocked = false;
                }));
        //yes to whoever is reading this, i still need the commented code below
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
//                .whileActiveContinuous(new InstantCommand(() -> {
//                    if (1 >= launcherMotor && launcherMotor >= -1) {
//                        launcherMotor -= 0.1;
//                    }
//                }));
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
//                .whileActiveContinuous(new InstantCommand(() -> {
//                    if (1 >= launcherMotor && launcherMotor >= -1) {
//                        launcherMotor += 0.1;
//                    }
//                }));
        telemetry.addLine("READY!");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        double forward = drive.getLeftY();
        double strafe  =  drive.getLeftX();
        double rotate  =  drive.getRightX();
        LLResult result = limelight.getLatestResult();


        if (isHeadingLocked) {


                double error = result.getTxNC();


                if (result.isValid()) {
                    //finalRotation = (error * HEADING_KP_TX) + (integralSum * HEADING_KI_TX) + (derivative * HEADING_KD_TX);
                    finalRotation = (error * HEADING_KP_TX);

                    if (Math.abs(finalRotation) > 0 && Math.abs(finalRotation) < ROTATION_MIN_POWER) {
                        finalRotation = Math.signum(finalRotation) * ROTATION_MIN_POWER;
                    }
                }
                else{
                    finalRotation = 0;
                }

            //intake automations
            double theta = Math.atan2(forward, strafe);
            double r = Math.hypot(forward, strafe);
            theta = AngleUnit.normalizeRadians(theta - robotHeading);

            double newForward = r * Math.sin(theta);
            double newStrafe  = r * Math.cos(theta);
            drive.drive(newForward, newStrafe, finalRotation);
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();

            telemetry.addData("Battery Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());

            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());
            telemetry.addData("Heading Lock", isHeadingLocked ? "ON" : "OFF");
            //telemetry.addData("test Variable", testVariable);
            telemetry.addData("Path timer: ", pathTimer.getElapsedTimeSeconds());
            telemetry.addData("test: ", testVariable);
            telemetry.addData("rotation", finalRotation);
            telemetry.update();

        }
    }
}
