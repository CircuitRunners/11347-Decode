package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.subsystems.intake
import org.firstinspires.ftc.teamcode.subsystems.outtake

class TransferCommand(
    private var inSubsystem: intake,
    private var out: outtake,
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
