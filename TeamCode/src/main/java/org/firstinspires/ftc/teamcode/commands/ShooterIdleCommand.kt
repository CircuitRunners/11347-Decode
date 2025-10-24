package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter

class ShooterIdleCommand(private val shooter: StaticShooter) : CommandBase() {
    init {
        addRequirements(shooter)
    }

    override fun execute() {
        shooter.setTargetRPM(0.0)
    }

    override fun isFinished() = false
}
