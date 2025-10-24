package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.intake
import org.firstinspires.ftc.teamcode.subsystems.outtake
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys

class IntakeCommand(
    private val inSubsystem: intake,
    private val out: outtake,
    private val driver: GamepadEx
) : CommandBase() {


    init {
        addRequirements(inSubsystem, out)
    }

    override fun execute() {
        val leftTrigger = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        val rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)

        if (leftTrigger > 0.1 || rightTrigger > 0.1) {
            out.block()
            inSubsystem.runIntake(leftTrigger - rightTrigger)
            inSubsystem.runTransfer(-0.5)
        } else {
            inSubsystem.stop()
            out.block()
        }
    }

    override fun end(interrupted: Boolean) {
        inSubsystem.stop()
        out.block()
    }
}
