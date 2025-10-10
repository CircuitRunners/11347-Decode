package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake extends SubsystemBase {
    public DcMotorEx continiousOuttake;
    public Servo blockingServo;
    public Servo blockingServoTwo;
    public Servo aimingServo;

    public double blockingPosition = 0;
    public double blockingPositionTwo = 0;
    public double nonBlockingPosition=1;
    public double nonBlockingPositionTwo=1;


    public outtake(HardwareMap hardwareMap) {
        continiousOuttake = hardwareMap.get(DcMotorEx.class, "outtake");
        blockingServo= hardwareMap.get(Servo.class, "blockingServoRight");
        blockingServoTwo= hardwareMap.get(Servo.class, "blockingServoLeft");
        aimingServo = hardwareMap.get(Servo.class, "launchingServo");

}
    public void setContiniousOuttake(double power) {
        continiousOuttake.setPower(power);



        }
    public void setBlock () {
        blockingServo.setPosition(blockingPosition);
        blockingServoTwo.setPosition(blockingPositionTwo);

    }
    public void nonBlock(){
        blockingServo.setPosition(nonBlockingPosition);
        blockingServoTwo.setPosition(nonBlockingPositionTwo);
    }

    public void aiming ( boolean isUp, boolean isDown)
        {double currentPosition = aimingServo.getPosition();
            aimingServo.setPosition(currentPosition+0.11);
            if (isUp){
                aimingServo.setPosition(currentPosition+0.11);

            }
                else if (isDown){
                    aimingServo.setPosition(currentPosition-0.11);

            }

        }
}