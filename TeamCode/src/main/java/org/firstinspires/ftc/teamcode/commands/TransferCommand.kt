package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter

class TransferCommand(
    private var inSubsystem: IntakeSubsystem,
    private var out: OuttakeSubsystem,
    private var driver: GamepadEx,
    private var shooter : StaticShooter
) : CommandBase() {
    init {
        addRequirements(inSubsystem, out)
    }

    override fun execute() {
        if (driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()) {
            if (shooter.targetRPM > 4000) {
                if (shooter.getShooterVelocity() > 4200) {
                    inSubsystem.transfer(0.75)
                    out.unblock();
                }
                else{
                    out.block()
                }
            } else {
                inSubsystem.transfer()
                out.unblock();
            }
            //inSubsystem.transfer(0.75)
            //out.unblock();
        } else {
            inSubsystem.stop()
            out.block()
        }
    }

    override fun end(interrupted: Boolean) {
        inSubsystem.stop()
        out.block()
    }

    override fun isFinished() =
        !driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()
}
