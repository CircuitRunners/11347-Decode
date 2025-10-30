package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BeamBreakHelper extends SubsystemBase {
    private DigitalChannel intakeBeamBreak;
    private ElapsedTime beamTimer = new ElapsedTime();
    private static final double BEAM_HOLD_TIME = 0.35;
    private boolean isBeamStable = false;

    /**
     * Constructor for the Beam Break Helper Class
     * @param hardwareMap gets the hardware map from the calling class to initialize devices
     */
    public BeamBreakHelper(HardwareMap hardwareMap) {
        intakeBeamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        intakeBeamBreak.setMode(DigitalChannel.Mode.INPUT);
    }

    ///  Updates the beamBreak every loop
    public void update() {
        boolean isBeamBroken = !getBeamState(); // false means the beam is broken
        if (isBeamBroken) {
            if (beamTimer.seconds() >= BEAM_HOLD_TIME) isBeamStable = true;
        } else {
            beamTimer.reset();
            isBeamStable = false;
        }
    }

    /**
     * Checks if the beam is stable in the off state
     * @return returns true if the beam IS broken for more than BEAM_HOLD_TIME
     *         returns false if the beam IS NOT broken for more than BEAM_HOLD_TIME
     */
    public boolean isBeamStable() {
        return isBeamStable;
    }

    /**
     * Checks if the beam is currently broken or not
     * @return returns false if the beam IS broken,
     *         returns true if the beam IS NOT broken
     */
    public boolean getBeamState() {
        return intakeBeamBreak.getState();
    }
}
