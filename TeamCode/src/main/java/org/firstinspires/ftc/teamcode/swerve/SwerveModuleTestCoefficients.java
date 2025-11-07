package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SwerveModuleTestCoefficients {
    public final double WHEEL_CIRCUMFERENCE; // Wheel circumference in meters
    public final double TICKS_PER_REVOLUTION;

    public PIDFCoefficients drivePIDFCoefficients, anglePIDCoefficients;

    /**
     * Constructor for SwerveDriveCoefficients.
     *
     * @param wheelCircumference    Wheel circumference in meters
     * @param ticksPerRevolution    Ticks per revolution of the wheel encoder
     * @param drivePIDFCoefficients PIDF coefficients for the drive system
     */
    public SwerveModuleTestCoefficients(double wheelCircumference, double ticksPerRevolution,
                                        PIDFCoefficients drivePIDFCoefficients,
                                        PIDFCoefficients anglePIDFCoefficients
    ) {
        WHEEL_CIRCUMFERENCE = wheelCircumference;
        TICKS_PER_REVOLUTION = ticksPerRevolution;

        this.drivePIDFCoefficients = drivePIDFCoefficients;
        this.anglePIDCoefficients = anglePIDFCoefficients;

    }
}
