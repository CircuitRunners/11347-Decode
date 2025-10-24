package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * StaticShooter subsystem controls a single flywheel-style shooter.
 * Provides PIDF tuning, velocity targeting, and telemetry integration with FTC Dashboard.
 */
@Config
public class StaticShooter extends SubsystemBase {
    // --- Hardware ---
    private DcMotorEx shooter;

    // --- Dashboard & Telemetry ---
    private FtcDashboard dash;

    // --- Shooter Constants ---
    private static double TARGET_RPM = 3500.0;         // desired shooter RPM
    private static double MOTOR_RPM = 1620.0;          // motor RPM (based on max motor rpm)
    private static double GEAR_RATIO = 2.5;            // gear ratio from motor to shooter
    private static double TICKS_PER_REV = 103.8;       // motor encoder ticks per revolution

    // --- PIDF Coefficients ---
    //working value 35 on October 9th, 2025
/*    public double kP = 35.0;
    public double kI = 0.0;
    public double kD = 10.0;
    public double kF = 13.0;*/
    public static double kP = 35.0;
    public static double kI = 0.0;
    public static double kD = 10.0;
    public static double kF = 13.0;

    /**
     * Initialises the shooter in the hardwareMap, sets default shooter values
     * @param hardwareMap pulls HardwareMap from teleOp class
     *                    to initialise motor
     * @param telemetry Allows the class to add telemetry to the phone
     * @param defaultTargetRPM Sets the default target RPM of the shooter
     *                        Set to the initial target RPM of your
     *                        shooter
     * @param defaultMotorRPM Sets the default RPM of the motor
     *                       Set to the RPM of the motor being used
     * @param defaultGearRatio Sets the default shooter gear ratio
     *                        Set to the gear ratio between the motor and shooterwheel
     * @param defaultTicks Sets the default ticks of the motor
     *                    Set to the encoder ticks of your motor
     */
    public StaticShooter(HardwareMap hardwareMap, Telemetry telemetry, double defaultTargetRPM,
                         double defaultMotorRPM, double defaultGearRatio, double defaultTicks) {
        // Initializes dashboard telemetry
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        // Configs shooter
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configs defaults
        setTargetRPM(defaultTargetRPM);
        setMotorRPM(defaultMotorRPM);
        setGearRatio(defaultGearRatio);
        setTicksPerRev(defaultTicks);

        // Apply initial PIDF coefficients
        applyPIDF();

        telemetry.addLine("shooter Init Done");
    }

    /// Only use if the constants in this file are correct
    public StaticShooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, TARGET_RPM, MOTOR_RPM, GEAR_RATIO, TICKS_PER_REV);
    }

    // --- PIDF ---
    /**
     * Sets shooter PIDF coefficients manually
     * @param kf Set to a low value, just enough that the shooter wheel
     *           begins to rotate
     * @param kp Increase kP after kF until the shooter wheel reaches the target speed
     * @param kd Try changing the target speed of the shooter from a low value
     *           to a high value and vise versa. Use this to reduce the
     *           oscillations when changing speeds
     * @param ki Most times this won't need to be tuned
     */
    public void setShooterPIDF(double kf, double kp, double kd, double ki) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        applyPIDF();
    }

    /** Applies current shooter velocity PIDF coefficients */
    public void applyPIDF() {
        shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    // --- Constants Control ---
    /**
     * Changes the target RPM of the shooter
     * @param targetRPM Set to the target RPM of the shooter
     */
    public void setTargetRPM(double targetRPM) {
        TARGET_RPM = targetRPM;
    }

    /**
     * Changes the RPM of the motor
     * @param motorRPM Set to the RPM of the motor
     *
     */
    public void setMotorRPM(double motorRPM) {
        MOTOR_RPM = motorRPM;
    }

    /**
     * Changes the gear ratio between the motor and the shooter
     * @param gearRatio Set to the gear ratio used between the
     *                  motor and shooter
     *      1.0 is a 1:1 gear ratio
     *      2.5 is a 2.5:1 gear increase
     *      0.5 is a 0.5:1 gear reduction
     */
    public void setGearRatio(double gearRatio) {
        GEAR_RATIO = gearRatio;
    }

    public double getGearRatio() {
        return GEAR_RATIO;
    }

    /**
     * Changes the Ticks Per Revolution of the motor
     * Called Encoder Resolution on gobilda website
     * @param TicksPerRev Set to the Ticks per rev of the motor
     *                    being used
     */
    public void setTicksPerRev(double TicksPerRev) {
        TICKS_PER_REV = TicksPerRev;
    }

    public double getTicksPerRev() {
        return TICKS_PER_REV;
    }

    /**
     * Calculates ticks per second based on target RPM
     * Sets the target velocity
     * */
    public void runShooter() {
        double targetTicksPerSec = ((TARGET_RPM / GEAR_RATIO) * TICKS_PER_REV) / 60;
        shooter.setVelocity(targetTicksPerSec);
    }

    /** Stops all shooter motion immediately. */
    public void eStop() {
        shooter.setPower(0);
        shooter.setVelocity(0);
    }

    /**
     * Gets shooter current velocity
     * @return Returns current shooter RPM based on the
     *         motor rpm, ticks per rev, and gear ratio
     */
    public double getShooterVelocity() {
        double currTicksPerSec = shooter.getVelocity(); // ticks/s of motor
        double currMotorRPM = (currTicksPerSec * 60.0) / TICKS_PER_REV;
        double currShooterRPM = currMotorRPM * GEAR_RATIO;

        return currShooterRPM;
    }

    /**
     * Gets shooter motor current velocity
     * @return Returns motor voltage
     */
    public double getMotorVoltage() {
        return shooter.getCurrent(CurrentUnit.AMPS);
    }
}
