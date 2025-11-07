package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.support.SRSHub;

public class SRSConstants {
    public int i2cPort = 1;
    public float forwardPodX = -28.042f;
    public float strafePodY = -147.012f;
    public float encoderResolution = 19.89436789f;
    public SRSHub.GoBildaPinpoint.EncoderDirection forwardEncoderDirection = SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD;
    public SRSHub.GoBildaPinpoint.EncoderDirection strafeEncoderDirection = SRSHub.GoBildaPinpoint.EncoderDirection.REVERSED;
    public DistanceUnit distanceUnit = DistanceUnit.MM;
    public String hardwareMapName = "srsHub";

    public SRSConstants i2cPort(int port) {
        this.i2cPort = port;
        return this;
    }

    public SRSConstants forwardPodY(float val) {
        this.forwardPodX = val;
        return this;
    }

    public SRSConstants strafePodX(float val) {
        this.strafePodY = val;
        return this;
    }

    public SRSConstants encoderResolution(float val) {
        this.encoderResolution = val;
        return this;
    }

    public SRSConstants forwardEncoderDirection(SRSHub.GoBildaPinpoint.EncoderDirection dir) {
        this.forwardEncoderDirection = dir;
        return this;
    }

    public SRSConstants strafeEncoderDirection(SRSHub.GoBildaPinpoint.EncoderDirection dir) {
        this.strafeEncoderDirection = dir;
        return this;
    }

    public SRSConstants distanceUnit(DistanceUnit unit) {
        this.distanceUnit = unit;
        return this;
    }

    public SRSConstants hardwareMapName(String name) {
        this.hardwareMapName = name;
        return this;
    }
}
