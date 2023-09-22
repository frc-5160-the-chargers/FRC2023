package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.buildCommand
import frc.chargers.commands.drivetrainCommands.driveStraight
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.robot.commands.scoreLow
import frc.robot.hardware.subsystems.Arm
import frc.robot.hardware.subsystems.Intake

/**
 * A command that scores on the low goal before driving out, earning taxi and scoring points.
 * Uses the [buildCommand] DSL.
 */
context(HeadingProvider)
fun EncoderDifferentialDrivetrain.scoreTaxi(arm: Arm, intake: Intake): Command = buildCommand {
    +arm.scoreLow(intake)

    this@scoreTaxi.driveStraight(-3.5.meters, -0.2, PIDConstants(0.04, 0.0, 0.0))
}
