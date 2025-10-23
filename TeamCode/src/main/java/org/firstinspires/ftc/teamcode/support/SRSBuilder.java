package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * SRSBuilder - Fluent builder utility for constructing and initializing SRSHub configurations.
 * Makes it easy to add encoders, I2C devices, and analog/digital inputs without manual Config wiring.
 */
public class SRSBuilder {

    private final SRSHub.Config config;
    private final HardwareMap hardwareMap;
    private final String SRSHubName;

    /**
     * Creates a new SRSBuilder for constructing an SRSHub.
     * @param hardwareMap The FTC HardwareMap to retrieve the SRSHub from.
     */
    public SRSBuilder(HardwareMap hardwareMap, String SRSHubName) {
        this.hardwareMap = hardwareMap;
        this.SRSHubName = SRSHubName;
        this.config = new SRSHub.Config();
    }

    /**
     * Adds a quadrature or PWM encoder to the SRSHub.
     * @param port The encoder port (1–6)
     * @param type The encoder type (SRSHub.Encoder.QUADRATURE or SRSHub.Encoder.PWM)
     * @return The builder instance for chaining.
     */
    public SRSBuilder addEncoder(int port, SRSHub.Encoder type) {
        config.setEncoder(port, type);
        return this;
    }

    /**
     * Adds a GoBILDA Pinpoint odometry device to an I2C bus.
     * @param bus The I2C bus number (1–3)
     * @param xOffset mm offset of forward pod from robot center
     * @param yOffset mm offset of strafe pod from robot center
     * @param encoderResolution ticks per mm of encoder
     * @param xDir Direction of X encoder (FORWARD or REVERSED)
     * @param yDir Direction of Y encoder (FORWARD or REVERSED)
     * @return The builder instance for chaining.
     */
    public SRSBuilder addPinpoint(int bus,
                                  float xOffset,
                                  float yOffset,
                                  float encoderResolution,
                                  SRSHub.GoBildaPinpoint.EncoderDirection xDir,
                                  SRSHub.GoBildaPinpoint.EncoderDirection yDir) {

        config.addI2CDevice(bus, new SRSHub.GoBildaPinpoint(
                xOffset, yOffset, encoderResolution, xDir, yDir
        ));
        return this;
    }

    /**
     * Adds a general I2C device to the SRSHub (useful for distance sensors, color sensors, etc.)
     * @param bus The I2C bus number (1–3)
     * @param device The SRSHub.I2CDevice subclass (e.g., VL53L0X or APDS9151)
     * @return The builder instance for chaining.
     */
    public SRSBuilder addI2CDevice(int bus, SRSHub.I2CDevice device) {
        config.addI2CDevice(bus, device);
        return this;
    }

    /**
     * Adds an analog or digital device to the SRSHub.
     * @param pin The pin number (1–12)
     * @param deviceType The device type (SRSHub.AnalogDigitalDevice.ANALOG, DIGITAL, or NONE)
     * @return The builder instance for chaining.
     */
    public SRSBuilder addAnalogDigital(int pin, SRSHub.AnalogDigitalDevice deviceType) {
        config.setAnalogDigitalDevice(pin, deviceType);
        return this;
    }

    /**
     * Finalizes configuration, initializes the hub, and returns the SRSHub instance.
     * @return Initialized SRSHub ready for use.
     */
    public SRSHub build() {
        SRSHub hub = hardwareMap.get(SRSHub.class, SRSHubName);
        hub.init(config);
        return hub;
    }

    /**
     * Exposes the current SRSHub.Config without initializing (for advanced use).
     */
    public SRSHub.Config getConfig() {
        return config;
    }
}
