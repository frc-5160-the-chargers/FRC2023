package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.buildCommand
import frc.chargers.commands.drivetrainCommands.driveStraight
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain


context(HeadingProvider)
fun EncoderDifferentialDrivetrain.driveBack(): Command = buildCommand {
    this@driveBack.driveStraight(3.5.meters, 0.2, PIDConstants(0.04, 0.0, 0.0))
}