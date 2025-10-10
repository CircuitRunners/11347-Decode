package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.StaticShooter;

/**
 * ShooterCommandOpMode
 * --------------------
 * Command-based test OpMode for the StaticShooter subsystem.
 *
 * Controls:
 *  - A: Start shooter
 *  - B: Stop shooter
 *  - D-Pad Up: Increase target RPM (+100)
 *  - D-Pad Down: Decrease target RPM (−100)
 */
@TeleOp(name = "Shooter Command TeleOp", group = "Testing")
public class ShooterTestTeleOp extends CommandOpMode {

    private StaticShooter shooter;
    private GamepadEx driver;
    private boolean shooterRunning = false;
    private double lastDpadTime = 0.0;
    private final double DEBOUNCE = 0.3; // seconds
    private double curr;

    @Override
    public void initialize() {
        telemetry.addLine("Initializing ShooterCommandOpMode...");
        telemetry.update();

        // --- Initialize Shooter Subsystem ---
        shooter = new StaticShooter(hardwareMap, telemetry);
        curr = shooter.getTargetRPM();

        // --- Gamepad Controls ---
        // Start shooter
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    shooterRunning = true;
                    telemetry.addLine("Shooter started.");
                })
        );

        // Stop shooter
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    shooter.eStop();
                    shooterRunning = false;
                    telemetry.addLine("Shooter stopped.");
                })
        );

        // Increase target RPM (D-pad up)
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    double now = time;
                    if (now - lastDpadTime > DEBOUNCE) {
                        shooter.setTargetRPM(curr += 100);
                        lastDpadTime = now;
                        telemetry.addData("RPM Adjust", "+100");
                    }
                })
        );

        // Decrease target RPM (D-pad down)
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    double now = time;
                    if (now - lastDpadTime > DEBOUNCE) {
                        shooter.setTargetRPM(Math.max(0, curr - 100));
                        lastDpadTime = now;
                        telemetry.addData("RPM Adjust", "-100");
                    }
                })
        );

        telemetry.addLine("Init Done! Press A to start shooter.");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("Shooter Running", shooterRunning);
        telemetry.addData("Target RPM", curr);
        telemetry.addData("Current RPM", shooter.getShooterVelocity());
        telemetry.addData("Motor Voltage (Amps)", shooter.getMotorVoltage());
        telemetry.update();
    }
}
