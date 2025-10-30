package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public void setAllianceTagID(int allianceTagID) {
        this.allianceTagID = allianceTagID;
    }

    public int getLimelightAllianceTagID() {
        return allianceTagID;
    }

    public void update() {
        latestResult = limelight.getLatestResult();
    }

    public boolean hasValidTarget() {
        if (latestResult == null || !latestResult.isValid()) return false;
        for (LLResultTypes.FiducialResult fr : latestResult.getFiducialResults()) {
            if (fr.getFiducialId() == allianceTagID) return true;
        }
        return false;
    }

    public double getTx() {
        return (latestResult != null) ? latestResult.getTxNC() : 0.0;
    }

    public double getTy() {
        return (latestResult != null) ? latestResult.getTyNC() : 0.0;
    }

    public double getDistanceToTagCenterInches(boolean groundPlane) {
        if (latestResult == null || !latestResult.isValid()) return 0.0;

        for (LLResultTypes.FiducialResult fr : latestResult.getFiducialResults()) {
            if (fr.getFiducialId() == allianceTagID) {
                Pose3D pose = fr.getCameraPoseTargetSpace();
                if (pose == null) return 0.0;

                // Pose3D -> Position -> meters
                Position pos = pose.getPosition();
                double x = pos.x; // Right (+)
                double y = pos.y; // Down (+)
                double z = pos.z; // Forward (+)

                double meters = groundPlane
                        ? Math.hypot(x, z) // X–Z plane distance
                        : Math.sqrt(x * x + y * y + z * z); // 3D distance

                return meters * 39.3701; // Convert meters → inches
            }
        }
        return 0.0;
    }


    public LLResult getLatest() {
        return latestResult;
    }

    public LLResultTypes.FiducialResult getDesiredFiducial() {
        if (latestResult == null || !latestResult.isValid()) return null;
        for (LLResultTypes.FiducialResult fr : latestResult.getFiducialResults()) {
            if (fr.getFiducialId() == allianceTagID) return fr;
        }
        return null;
    }

    public void stop() {
        limelight.stop();
    }
}
