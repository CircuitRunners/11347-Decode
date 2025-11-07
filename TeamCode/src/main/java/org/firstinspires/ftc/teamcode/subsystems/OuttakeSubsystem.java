package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.support.RunAction;

public class OuttakeSubsystem extends SubsystemBase {
    // === Enums ===
    public enum BlockState {
        BLOCK(0.5, 0.5),
        UNBLOCK(0.32, 0.32);

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
        AIM_MAX(0.25),
        AIM_MIN(0.00);

        public final double position;
        AimState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public RunAction block, unblock, aimMin, aimMax;
    public Servo blockingServoLeft, blockingServoRight;
    public Servo aimingServo;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        blockingServoLeft = hardwareMap.get(Servo.class, "blockingLeft");
        blockingServoRight = hardwareMap.get(Servo.class, "blockingRight");
        aimingServo = hardwareMap.get(Servo.class, "aimServo");
        aimingServo.setDirection(Servo.Direction.REVERSE);
        blockingServoLeft.setDirection(Servo.Direction.REVERSE);

        blockingServoLeft.setPosition(BlockState.BLOCK.getLeft());
        blockingServoRight.setPosition(BlockState.BLOCK.getRight());
        aimClose();

        block = new RunAction(this::block);
        unblock = new RunAction(this::unblock);
        aimMin = new RunAction(this::aimMin);
        aimMax = new RunAction(this::aimMax);
    }

    public double getBlockPosLeft() {
        return blockingServoLeft.getPosition();
    }

    public double getBlockPosRight() {
       return blockingServoRight.getPosition();
    }

    public double getAimPos() {
        return aimingServo.getPosition();
    }

    public void block() {
        setBlockPosition(BlockState.BLOCK);
    }

    public void unblock() {
        setBlockPosition(BlockState.UNBLOCK);
    }

    public void aimServo(double value) {
        blockingServoLeft.setPosition(value);
    }


    public void setBlockPosition(BlockState state) {
        blockingServoLeft.setPosition(state.getLeft());
        blockingServoRight.setPosition(state.getRight());
    }

    public void aimMin() {
        aimingServo.setPosition(AimState.AIM_MIN.getPosition());
    }

    public void aimMax() {
        aimingServo.setPosition(AimState.AIM_MAX.getPosition());
    }

    public void aimScoring() {
        setAim(0.28);
    }

    public void aimClose() {
        setAim(0.19);
    }

    public void setAim(double position) {
        aimingServo.setPosition(position);
    }

    public void aiming(boolean up, boolean down) {
        double currentPos = aimingServo.getPosition();

        if (up) {
            aimingServo.setPosition(Range.clip(currentPos-0.005, 0, 04.5));
        } else if (down) {
            aimingServo.setPosition(Range.clip(currentPos+0.005, 0, 04.5));
        }
    }
}
