/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name = "SwerveTest", group = "Test")
//@Disabled
@Config
public class SwerveModuleTest extends CommandOpMode {

    // Declare OpMode members.

    PIDFController angleController;

    public static double p = 0.02, i = 0, d = 0.0006, f = 0;

    double pidout = 0;
    private CRServo angleServo;
    /**
     * The motor responsible for driving the module.
     */
    private DcMotor driveMotor;
    /**
     * The encoder used to determine the absolute angle of the module.
     */
    private AnalogInput servoPotentiometer;
    private double targetAngle;
    private double lastTargetAngle = -1;
    private double angleError;
    private double targetMinDistance;
    private double angleOffset = 35;

    @Override
    public void initialize() {

    }

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        angleController = new PIDFController(p, i, d, f);


        driveMotor = hardwareMap.get(DcMotor.class, "driveMotor");
        angleServo = hardwareMap.get(CRServo.class, "angleServo");
        servoPotentiometer = hardwareMap.get(AnalogInput.class, "angleInput");

        angleServo.setDirection(DcMotorSimple.Direction.REVERSE);
        angleController.setSetPoint(getAngleDegrees());
        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper)
                driveMotor.setPower(gamepad1.left_stick_x);
            else driveMotor.setPower(0);

            if (gamepad1.right_bumper) {
                angleServo.setPower(gamepad1.right_stick_x);
                targetAngle = getAngleDegrees();
            }

            if (!gamepad1.right_bumper) {
                if (targetAngle != lastTargetAngle + 3 || targetAngle != lastTargetAngle - 3) {
                    double errorBound = 180;
                    targetMinDistance =
                            inputModulus(targetAngle - getAngleDegrees(), -errorBound, errorBound);
                    angleController.setSetPoint(getAngleDegrees() + targetMinDistance);
                    lastTargetAngle = targetAngle;
                }

                pidout = angleController.calculate(getAngleDegrees());
                if (pidout > 1) pidout = 1;
                if (pidout < -1) pidout = -1;

                angleServo.setPower(pidout);
            }


            if (gamepad1.a) targetAngle = 0;
            if (gamepad1.b) targetAngle = 90;
            if (gamepad1.x) targetAngle = 180;
            if (gamepad1.y) targetAngle = -90;


            angleController.setSetPoint(targetAngle);


            //  testModule.closeAngleLoop();


            // run until the end of the match (driver presses STOP)


            telemetry.addData("Cmd", gamepad1.right_stick_x);
            telemetry.addData("ContError", angleController.getPositionError());
            telemetry.addData("ContTgt", angleController.getSetPoint());

            telemetry.addData("AngleError", angleError);
            telemetry.addData("MinDistance", targetMinDistance);
            telemetry.addData("PIDError", pidout);

            telemetry.addData("ServoPotRaw", servoPotentiometer.getVoltage());
            telemetry.addData("ServoAngle", getAngleDegrees());
            telemetry.addData("TargetAngle", targetAngle);

            telemetry.update();
        }
    }


    double getAngleDegrees() {
        double volts = servoPotentiometer.getVoltage();
        double potAngle = volts * 360 / 3.3;
        return Math.IEEEremainder((potAngle + angleOffset), 360);
    }

    double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }
}
