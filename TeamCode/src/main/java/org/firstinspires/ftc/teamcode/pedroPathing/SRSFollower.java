package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;

/**
 * Custom Follower subclass that ensures SRSLocalizer and PoseTracker start aligned.
 */
public class SRSFollower extends Follower {

    public SRSFollower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain, PathConstraints pathConstraints) {
        super(constants, localizer, drivetrain, pathConstraints);
    }

    @Override
    public void setStartingPose(Pose pose) {
        getPoseTracker().getLocalizer().setStartPose(pose);
        getPoseTracker().setPose(pose);
    }
}
