package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.support.RunAction;

public class outtake extends SubsystemBase {
    // === Enums ===
    public enum BlockState {
        BLOCK(0, 0),
        UNBLOCK(1, 1);

        public final double left, right;
        BlockState(double left, double right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft() {
            return left;
        }

        public double getRight() {
            return right;
        }
    }

    public enum AimState {
        AIM_MAX(1),
        AIM_MIN(0);

        public final double position;
        AimState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public RunAction block, unblock;
    public Servo blockingServoLeft, blockingServoRight;
    public Servo aimingServo;

    public outtake(HardwareMap hardwareMap) {
        blockingServoLeft = hardwareMap.get(Servo.class, "blockingLeft");
        blockingServoRight = hardwareMap.get(Servo.class, "blockingRight");
        aimingServo = hardwareMap.get(Servo.class, "launchingServo");

        blockingServoLeft.setPosition(BlockState.BLOCK.getLeft());
        blockingServoRight.setPosition(BlockState.BLOCK.getRight());

        aimingServo.setPosition(AimState.AIM_MIN.getPosition());
        block = new RunAction(this::block);
        unblock = new RunAction(this::unblock);
    }

    public void block() {
        setBlockPosition(BlockState.BLOCK);
    }

    public void unblock() {
        setBlockPosition(BlockState.UNBLOCK);
    }

    public void setBlockPosition(BlockState state) {
        blockingServoLeft.setPosition(state.getLeft());
        blockingServoRight.setPosition(state.getRight());
    }
}
