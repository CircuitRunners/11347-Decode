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

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.swerve.module.AxonSwerveModule;
import org.firstinspires.ftc.teamcode.swerve.module.SwerveModuleConfiguration;


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

@TeleOp(name = "SingleSwerveModule", group = "Test")
public class SingleSwerveModule extends CommandOpMode {

    // Declare OpMode members.
    AxonSwerveModule testModule;
    SwerveModuleConfiguration testConfig;
    PIDFController driveController;
    PIDFController angleController;
    PIDFCoefficients driveCoefficients;
    PIDFCoefficients angleCoefficients;

    SwerveModuleTestCoefficients coefficients;
    double driveWheelDiameter = 2;//inches
    double ticksPerRevolution = 200;


    @Override
    public void initialize() {
        driveCoefficients = new PIDFCoefficients(1, 0, 0, 0);
        angleCoefficients = new PIDFCoefficients(1, 0, 0, 0);

        coefficients = new SwerveModuleTestCoefficients(ticksPerRevolution, driveWheelDiameter * 2 * Math.PI, driveCoefficients, angleCoefficients);
        driveController = new PIDFController(0, 0, 0, 0);
        angleController = new PIDFController(0, 0, 0, 0);

        testConfig = new SwerveModuleConfiguration(driveController, angleController,
                "driveMotor", "angleServo", "servoAnalog");

        testModule = new AxonSwerveModule(testConfig, coefficients, hardwareMap);
    }

    @Override
    public void runOpMode() {

        double startAngle = testModule.getWheelAngleRad();


        angleController.setSetPoint(startAngle);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses START)
        waitForStart();


        if (gamepad1.left_bumper)
            testModule.setDrivePower(gamepad1.left_stick_x);
        else testModule.setDrivePower(0);

        if (gamepad1.right_bumper)
            testModule.setAnglePower(gamepad1.right_stick_x);
        else testModule.setAnglePower(0);


        if (gamepad1.a) angleController.setSetPoint(Math.PI / 2);
        if (gamepad1.b) angleController.setSetPoint(0);
        if (gamepad1.x) angleController.setSetPoint(-Math.PI / 2);
        if (gamepad1.y) angleController.setSetPoint(Math.PI);

      //  testModule.closeAngleLoop();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Show the elapsed game time and wheel power.

            telemetry.update();
        }
    }

    double getAngle() {
        return testModule.getWheelAngleDeg();
    }


}
