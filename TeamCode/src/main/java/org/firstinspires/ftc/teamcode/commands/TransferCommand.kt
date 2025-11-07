package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

class TransferCommand(
    private var inSubsystem: IntakeSubsystem,
    private var out: OuttakeSubsystem,
    private var driver: GamepadEx
) : CommandBase() {
    private var blocking = true;

    init {
        addRequirements(inSubsystem, out)
    }

    override fun execute() {
        if (driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()) {
            inSubsystem.transfer()
            out.unblock();
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
