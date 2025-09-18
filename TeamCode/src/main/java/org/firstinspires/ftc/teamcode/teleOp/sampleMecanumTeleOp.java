package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.support.Constants.pinpointXOffset;
import static org.firstinspires.ftc.teamcode.support.Constants.pinpointYOffset;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.support.PinpointOdo;

@TeleOp
public class sampleMecanumTeleOp extends CommandOpMode {

    public DcMotorEx leftFront, rightFront, leftBack, rightBack;
    PinpointOdo odo;

    @Override
    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        odo = hardwareMap.get(PinpointOdo.class,"odo");
        odo.setOffsets(pinpointXOffset,pinpointYOffset);
        odo.setEncoderResolution(PinpointOdo.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(PinpointOdo.EncoderDirection.REVERSED,
                PinpointOdo.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void run() {
        super.run();
        odo.update();

        Pose2D pos = odo.getPosition();
        double botHeading = pos.getHeading(AngleUnit.RADIANS);
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(-botHeading) + y * Math.sin(-botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


        rotY = -rotY;
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        // Set the motor powers
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        if(gamepad1.right_stick_button) {
            odo.resetPosAndIMU();
        }

    }
}
