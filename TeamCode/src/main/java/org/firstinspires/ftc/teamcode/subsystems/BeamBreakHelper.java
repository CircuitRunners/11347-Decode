package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class BeamBreakHelper extends SubsystemBase {

    private final DigitalChannel beamBreak;

    // Stable beam detection
    private final ElapsedTime stableTimer = new ElapsedTime();
    public static double BEAM_HOLD_TIME = 0.45;
    private boolean stableBroken = false;

    // Ball counting
    private boolean lastBroken = false;
    private boolean seenClear = false;     // <-- FIX #1
    private int ballCount = 0;

    public static double BALL_TIMEOUT = 0.5;
    private final ElapsedTime timeoutTimer = new ElapsedTime();
    private boolean inTimeout = false;

    public BeamBreakHelper(HardwareMap hw, String name, int StartingBallCount) {
        beamBreak = hw.get(DigitalChannel.class, name);
        beamBreak.setMode(DigitalChannel.Mode.INPUT);
        ballCount = StartingBallCount;
    }

    public void update() {
        boolean broken = isBeamBroken();

        // --- Stable hold detection ---
        if (broken) {
            if (stableTimer.seconds() >= BEAM_HOLD_TIME)
                stableBroken = true;
        } else {
            stableTimer.reset();
            stableBroken = false;
        }

        // --- Mark that we have seen the beam clear at least once ---
        if (!broken) {
            seenClear = true;
        }

        // --- Timeout handling ---
        if (inTimeout && timeoutTimer.seconds() >= BALL_TIMEOUT)
            inTimeout = false;

        // --- Count a ball when: ---
        //   1. rising edge (broken && !lastBroken)
        //   2. beam was clear earlier this run (avoids preload issue)
        //   3. not in timeout
        if (broken && !lastBroken && seenClear && !inTimeout) {
            ballCount++;
            inTimeout = true;
            timeoutTimer.reset();
        }

        lastBroken = broken;
    }

    public boolean isBeamBroken() {
        return !beamBreak.getState();
    }

    public boolean isBeamStable() {
        return stableBroken;
    }

    public synchronized int getBallCount() {
        return ballCount;
    }

    public void resetBallCount() {
        ballCount = 0;
        seenClear = false;  // allow resetting preloaded state
        inTimeout = false;
    }
}
