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
        AIM_MAX(0.48),
        AIM_MIN(0.37);

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
        blockingServoLeft = hardwareMap.get(Sxrvo.class, "blockingLeft");
        blockingServoRight = hardwareMap.get(Servo.class, "blockingRight");
        aimingServo = hardwareMap.get(Servo.class, "aimServo");
        aimingServo.setDirection(Servo.Direction.REVERSE);
        blockingServoLeft.setDirection(Servo.Direction.REVERSE);

        blockingServoLeft.setPosition(BlockState.BLOCK.getLeft());
        blockingServoRight.setPosition(BlockState.BLOCK.getRight());
//        aimingServo.setPosition(AimState.AIM_MIN.getPosition());
        aimScoring();

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
        aimingServo.setPosition(0.3);
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
