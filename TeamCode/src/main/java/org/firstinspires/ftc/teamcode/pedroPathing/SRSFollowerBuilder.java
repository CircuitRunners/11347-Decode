package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SRSFollowerBuilder extends FollowerBuilder {
    private Localizer customLocalizer;

    public SRSFollowerBuilder(FollowerConstants constants, HardwareMap hardwareMap) {
        super(constants, hardwareMap);
    }

    public SRSFollowerBuilder withLocalizer(Localizer localizer) {
        this.customLocalizer = localizer;
        return this;
    }

    @Override
    public Follower build() {
        if (customLocalizer != null) setLocalizer(customLocalizer);
        return super.build();
    }
}
