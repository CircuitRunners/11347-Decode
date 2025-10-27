package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SRSFollowerBuilder extends FollowerBuilder {
    private final FollowerConstants constants;
    private PathConstraints constraints;
    private HardwareMap hardwareMap;
    private Localizer localizer;
    private Drivetrain drivetrain;

    public SRSFollowerBuilder(FollowerConstants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap);
        this.constants = constants;
        this.hardwareMap = hardwareMap;
        constraints = PathConstraints.defaultConstraints;
    }

    public SRSFollowerBuilder setLocalizer(Localizer localizer) {
        this.localizer = localizer;
        return this;
    }

    public SRSFollowerBuilder srsLocalizer(SRSConstants lConstants) {
        return setLocalizer(new SRSLocalizer(hardwareMap, lConstants));
    }

    public SRSFollowerBuilder setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        return this;
    }

    public SRSFollowerBuilder mecanumDrivetrain(MecanumConstants mecanumConstants) {
        return setDrivetrain(new Mecanum(hardwareMap, mecanumConstants));
    }

    public SRSFollowerBuilder pathConstraints(PathConstraints pathConstraints) {
        this.constraints = pathConstraints;
        PathConstraints.setDefaultConstraints(pathConstraints);
        return this;
    }

    @Override
    public SRSFollower build() {
        return new SRSFollower(constants, localizer, drivetrain, constraints);
    }
}
