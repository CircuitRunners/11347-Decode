package org.firstinspires.ftc.teamcode.swerve.module;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.swerve.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.swerve.SwerveModuleTestCoefficients;


/**
 * A SwerveModule implementation using a CRServo for the rotation axis and a DcMotorEx for driving.
 * This class handles the control and feedback for a single swerve module, including motor and encoder setup.
 */
public class AxonSwerveModule extends SwerveModule {

    /**
     * The servo responsible for rotating the module.
     */
    private final CRServo m_angleServo;

    /**
     * The motor responsible for driving the module.
     */
    private final DcMotorEx m_driveMotor;

    /**
     * The encoder used to determine the absolute angle of the module.
     */
    private final AbsoluteAnalogEncoder m_absoluteEncoder;

    /**
     * Constructs a new AxonSwerveModule instance.
     *
     * @param config       The module's configuration parameters.
     * @param coefficients The swerve drive coefficients used for PIDF control.
     * @param hw           The hardware map used to access the robot's hardware components.
     */
    public AxonSwerveModule(SwerveModuleConfiguration config, SwerveModuleTestCoefficients coefficients, HardwareMap hw) {
        super(config, coefficients, hw);

        m_driveMotor = hw.get(DcMotorEx.class, config.driveMotorName);
        m_angleServo = hw.get(CRServo.class, config.angleServoName);
        m_absoluteEncoder = new AbsoluteAnalogEncoder(hw.get(AnalogInput.class, config.absoluteEncoderName));

        // Configure the drive motor
        m_driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_driveMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_driveMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients.drivePIDFCoefficients);
    }

    @Override
    public void setPower(double drive, double angle) {

    }

    /**
     * Sets the power for both the drive motor and the rotation servo.
     *
     * @param drive The speed of the drive motor, in the range -1.0 to 1.0.
     */

    public void setDrivePower(double drive) {
        m_driveMotor.setPower(drive);
    }

    public void setAnglePower(double power) {
        m_angleServo.setPower(power);
    }



    public void closeAngleLoop(){
        m_angleServo.setPower(m_configuration.anglePIDFController.calculate(m_absoluteEncoder.getCurrentPosition()));
    }

    /**
     * Gets the current state of the swerve module.
     *
     * @return The current {@link SwerveModuleState}, which includes the speed in meters per second
     * and the angle as a {@link Rotation2d}.
     */
    @Override
    public SwerveModuleState getState() {
        double speedMetersPerSecond = getWheelSpeed();
        double angleRadians = getWheelAngleRad();
        return new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(angleRadians * 180 / Math.PI));
    }

    /**
     * Gets the current wheel speed.
     *
     * @return The wheel speed in meters per second.
     */
    @Override
    public double getWheelSpeed() {
        double motorTicksPerSecond = m_driveMotor.getVelocity();
        double wheelRevolutionsPerSecond = motorTicksPerSecond / TICKS_PER_REVOLUTION;
        return wheelRevolutionsPerSecond * WHEEL_CIRCUMFERENCE;
    }

    /**
     * Gets the current wheel angle in radians.
     *
     * @return The wheel angle in radians.
     */
    @Override
    public double getWheelAngleRad() {
        return getWheelAngleDeg() * Math.PI / 180;
    }

    /**
     * Gets the current wheel angle in degrees.
     *
     * @return The wheel angle in degrees.
     */
    @Override
    public double getWheelAngleDeg() {
        return m_absoluteEncoder.getCurrentPosition();
    }
}
