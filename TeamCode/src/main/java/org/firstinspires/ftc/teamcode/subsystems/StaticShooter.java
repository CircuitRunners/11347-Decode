package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * StaticShooter subsystem controls a single flywheel-style shooter.
 * Provides PIDF tuning, velocity targeting, and telemetry integration with FTC Dashboard.
 */
public class StaticShooter {
    // --- Hardware ---
    private DcMotorEx shooter;

    // --- Dashboard & Telemetry ---
    private FtcDashboard dash;
    private Telemetry telemetry;

    // --- Shooter Constants ---
    private double TARGET_RPM = 3500.0;         // desired shooter RPM
    private double MOTOR_RPM = 1620.0;          // motor RPM (based on max motor rpm)
    private double GEAR_RATIO = 2.5;            // gear ratio from motor to shooter
    private double TICKS_PER_REV = 103.8;       // motor encoder ticks per revolution

    // --- PIDF Coefficients ---
    public double kP = 35.0;
    public double kI = 0.0;
    public double kD = 10.0;
    public double kF = 13.0;

    /**
     * Initialises the shooter in the hardwareMap, sets default shooter values
     * @param hardwareMap           pulls HardwareMap from teleOp class
     *                              to initialise motor
     * @param defaultTargetRPM      sets the default target RPM of the
     *                              shooter
     * @param defaultGearRatio      sets the default shooter gear ratio
     * @param defaultTicks          sets the default ticks of the motor
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

    public StaticShooter(HardwareMap hardwareMap, Telemetry telemetry) {
        new StaticShooter(hardwareMap, telemetry, TARGET_RPM, MOTOR_RPM, GEAR_RATIO, TICKS_PER_REV);
    }

    // --- PIDF ---
    /**
     * Sets shooter PIDF coefficients manually
     * @param kp
     * @param ki
     * @param kd
     * @param kf
     */
    public void setShooterPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        applyPIDF();
    }

    /** Applies current shooter velocity PIDF coefficients */
    private void applyPIDF() {
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

    public double getTargetRPM() {
        return TARGET_RPM;
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

    /**
     * Changes the Ticks Per Revolution of the motor
     * Called Encoder Resolution on gobilda website
     * @param TicksPerRev Set to the Ticks per rev of the motor
     *                    being used
     */
    public void setTicksPerRev(double TicksPerRev) {
        TICKS_PER_REV = TicksPerRev;
    }

    /**
     * Calculates ticks per second based on target RPM
     * Sets the target velocity
     * */
    public void runShooter() {
        double targetTicksPerSec = ((TARGET_RPM / GEAR_RATIO) * TICKS_PER_REV) / 60;
        shooter.setVelocity(targetTicksPerSec);
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Target Ticks/sec", targetTicksPerSec);
        telemetry.addData("Actual Velocity", shooter.getVelocity());
    }

    /** Stops all shooter motion immediately. */
    public void eStop() {
        shooter.setPower(0);
        shooter.setVelocity(0);
        telemetry.addLine("Shooter Stopped!");
        telemetry.update();
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
