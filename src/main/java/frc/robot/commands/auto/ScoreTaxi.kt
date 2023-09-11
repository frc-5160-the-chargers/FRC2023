package frc.robot.commands.auto

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.buildCommand
import frc.chargers.commands.drivetrainCommands.driveStraight
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.robot.commands.moveToAngular
import frc.robot.hardware.subsystems.Arm
import frc.robot.hardware.subsystems.Intake

context(HeadingProvider)
fun EncoderDifferentialDrivetrain.scoreTaxi(arm: Arm, intake: Intake): Command = buildCommand {
    +arm.moveToAngular(thetaA = 60.degrees, thetaB = 9.degrees).withTimeout(2.5)

    loopFor(0.5.seconds, intake) {
        intake.setCustomPower(-0.25)
    }

    runParallelUntilOneFinishes {
        this@scoreTaxi.driveStraight(3.5.meters, 0.2, PIDConstants(0.04, 0.0, 0.0))

        +arm.moveToAngular(thetaA = 133.degrees, thetaB = 0.degrees)
    }
}
