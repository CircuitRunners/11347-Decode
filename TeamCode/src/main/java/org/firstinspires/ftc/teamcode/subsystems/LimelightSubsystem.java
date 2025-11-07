// ✨ FIXED VERSION
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class LimelightSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    private int allianceTagID = 20; // defaults to blue
    private LLResult latestResult = null;

    public LimelightSubsystem(HardwareMap hardwareMap, String deviceName) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void setAllianceTagID(int allianceTagID) { this.allianceTagID = allianceTagID; }

    public int getLimelightAllianceTagID() { return allianceTagID; }

    public void update() { latestResult = limelight.getLatestResult(); }

    public boolean hasValidTarget() {
        if (latestResult == null || !latestResult.isValid()) return false;
        return latestResult.getFiducialResults().stream()
                .anyMatch(fr -> fr.getFiducialId() == allianceTagID);
    }

    public double getTx() { return (latestResult != null) ? latestResult.getTxNC() : 0.0; }
    public double getTy() { return (latestResult != null) ? latestResult.getTyNC() : 0.0; }

    public double getDistanceToTagCenterInches(boolean groundPlane) {
        if (latestResult == null || !latestResult.isValid()) return 0.0;

        for (LLResultTypes.FiducialResult fr : latestResult.getFiducialResults()) {
            if (fr.getFiducialId() == allianceTagID && fr.getCameraPoseTargetSpace() != null) {
                Position pos = fr.getCameraPoseTargetSpace().getPosition();
                double meters = groundPlane
                        ? Math.hypot(pos.x, pos.z)
                        : Math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
                return meters * 39.37; // meters → inches
            }
        }
        return 0.0;
    }

    public void stop() { limelight.stop(); }

    // ✨ ADDED: Get the AprilTag height in pixels (Java 8 compatible)
    public double getTagHeightPixels() {
        if (latestResult == null || !latestResult.isValid()) return 0.0;

        for (LLResultTypes.FiducialResult fr : latestResult.getFiducialResults()) {
            if (fr.getFiducialId() == allianceTagID) {
                List<List<Double>> corners = fr.getTargetCorners(); // correct type
                if (corners == null || corners.size() < 4) return 0.0;

                double topY   = corners.get(0).get(1);
                double bottomY = corners.get(2).get(1);
                return Math.abs(topY - bottomY) * 240; // assuming normalized coords → ~240px height
            }
        }
        return 0.0;
    }

    // ✨ ADDED: Estimate distance using tag height
    public double getDistanceFromHeight(double actualTagHeightInches) {
        double tagHeight = getTagHeightPixels();
        if (tagHeight <= 0) return 0.0;

        double focalLength = 275.0; // ✨ tune this value for your camera
        return (actualTagHeightInches * focalLength) / tagHeight;
    }

    public LLResult getLatest() { return latestResult; }
}
