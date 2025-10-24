package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.support.SRSHub;

/**
 * SRSStaticShooter extends StaticShooter but uses an encoder connected through the SRS Hub
 * instead of the motor port encoder. It reads position/velocity from an encoder channel
 * on the SRS Hub and computes shooter RPM manually.
 */
public class SRSStaticShooter extends StaticShooter {

    private final SRSHub srsHub;
    private final int encoderPort;   // Which SRS encoder port (1-6)
    private int lastTicks = 0;
    private long lastTime = 0;
    private double currentRPM = 0.0;

    /**
     * Creates a shooter that uses an SRS Hub encoder as its velocity feedback device.
     *
     * @param hardwareMap FTC HardwareMap
     * @param telemetry   Telemetry instance
     * @param srsHub      Initialized SRSHub instance
     * @param encoderPort Encoder port on the SRS Hub (1–6)
     */
    public SRSStaticShooter(HardwareMap hardwareMap, Telemetry telemetry,
                            SRSHub srsHub, int encoderPort) {
        super(hardwareMap, telemetry);
        this.srsHub = srsHub;
        this.encoderPort = encoderPort;
    }

    /**
     * Update the shooter RPM from the SRS encoder reading.
     * Call this periodically (every loop).
     */
    public void updateRPM() {
        srsHub.update();
        SRSHub.PosVel encoder = srsHub.readEncoder(encoderPort);

        long now = System.nanoTime();
        if (lastTime != 0) {
            double dtSec = (now - lastTime) / 1e9;   // ns → s
            int deltaTicks = encoder.position - lastTicks;

            // ticks/s → RPM → shooter RPM
            double ticksPerSec = deltaTicks / dtSec;
            double motorRPM = (ticksPerSec * 60.0) / getTicksPerRev();
            currentRPM = motorRPM * getGearRatio();
        }

        lastTicks = encoder.position;
        lastTime = now;
    }

    @Override
    public double getShooterVelocity() {
        // Return the latest computed shooter RPM
        return currentRPM;
    }

    /**
     * Optionally: returns motor RPM instead of shooter wheel RPM
     */
    public double getMotorRPM() {
        return currentRPM / getGearRatio();
    }

    /**
     * Optional convenience to directly print current shooter data.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter RPM", "%.1f", currentRPM);
//        telemetry.addData("Target RPM", "%.1f", getTargetRPM());
    }
}
