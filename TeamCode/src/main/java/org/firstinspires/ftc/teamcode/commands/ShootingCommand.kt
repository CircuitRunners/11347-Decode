package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelRaceGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

/**
 * ShootCommand spins up the flywheel, waits for target RPM,
 * unblocks the outtake, and fires.
 */
class ShootingCommand(
    private val shooter: StaticShooter,
    private val out: OuttakeSubsystem,
    private val inSubsystem: IntakeSubsystem,
    private val targetRPM: Double,
    private val lowerBound: Double = targetRPM - 100,
    private val upperBound: Double = targetRPM + 100
) : ParallelRaceGroup() {

    constructor(shooter: StaticShooter, out: OuttakeSubsystem, inSubsystem: IntakeSubsystem, targetRPM: Double)
            : this(shooter, out, inSubsystem, targetRPM, targetRPM - 100, targetRPM + 100)

    init {
        addCommands(
            SequentialCommandGroup(
                InstantCommand({ shooter.setTargetRPM(targetRPM) }),
                WaitUntilCommand {
                    val velocity = shooter.getShooterVelocity()
                    velocity in lowerBound..upperBound
                },
                InstantCommand({ out.unblock() }),
                WaitCommand(500),
                InstantCommand({ inSubsystem.shoot() })
            )
        )
        addRequirements(shooter, out, inSubsystem)
    }
}
