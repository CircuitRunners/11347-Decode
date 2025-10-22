package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake extends SubsystemBase {
    //TODO: blocking, not required but helpful
    public enum BlockPosition {
        BLOCK(0, 0),
        UNBLOCK(1,1);

        public final double left, right;

        BlockPosition (double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    public Servo blockingServo;
    public Servo blockingServoTwo;
    public Servo aimingServo;
    public DcMotorEx outtakeMotor;

    public double blockingPosition = 0;
    public double blockingPositionTwo = 0;
    public double nonBlockingPosition=1;
    public double nonBlockingPositionTwo=1;

    public boolean isOuttake = false;

    public Outtake(HardwareMap hardwareMap) {
        blockingServo= hardwareMap.get(Servo.class, "blockRight");
        blockingServoTwo= hardwareMap.get(Servo.class, "blockLeft");
        aimingServo = hardwareMap.get(Servo.class, "hood");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        }
    public void setBlock () {
        blockingServo.setPosition(blockingPosition);
        blockingServoTwo.setPosition(blockingPositionTwo);

    }
    public void nonBlock(){
        blockingServo.setPosition(nonBlockingPosition);
        blockingServoTwo.setPosition(nonBlockingPositionTwo);
    }

    public void aiming ( boolean isUp, boolean isDown) {
        double currentPosition = aimingServo.getPosition();
            if (isUp){
                aimingServo.setPosition(currentPosition-0.001);

            }
                else if (isDown){
                    aimingServo.setPosition(currentPosition+0.001);

            }

    }
    public void setOutake(double power) {
            outtakeMotor.setPower(power);
    }

    public void toggleOuttake() {
        if (isOuttake) {
            isOuttake = !isOuttake;
            outtakeMotor.setPower(0);
        } else {
            isOuttake = !isOuttake;
            outtakeMotor.setPower(1);
        }
    }
}