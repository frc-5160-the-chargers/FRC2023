package frc.robot.commands.auto

import com.batterystaple.kmeasure.quantities.abs
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.feet
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.commands.buildCommand
import frc.chargers.commands.drivetrainCommands.driveStraight
import frc.chargers.constants.TurnPIDConstants
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.robot.commands.moveToAngular
import frc.robot.hardware.subsystems.Arm
import frc.robot.hardware.subsystems.Intake
import frc.robot.math.sin

private const val autoSpeed = 0.4
fun EncoderDifferentialDrivetrain.scoreTaxiBalance(arm: Arm, intake: Intake, navX: NavX) = buildCommand {
    runUntilFinish(arm.moveToAngular(thetaA = 60.degrees, thetaB = 9.degrees).withTimeout(2.0))

    runFor(0.5.seconds, intake) {
        intake.setCustomPower(-0.25)
    }

    runFor(0.4.seconds, arm) {
        arm.moveSpeeds(omegaA = 0.0, omegaB = -0.4)
        intake.setCustomPower(0.0)
    }

//    runUntilFinish(arm.moveToAngular(thetaA = 60.degrees, thetaB = 40.degrees).withTimeout(0.8))

    runParallelUntilOneFinishes {
        runUntil({ navX.gyroscope.pitch - 2.9.degrees < -10.degrees }, this@scoreTaxiBalance) {
            curvatureDrive(autoSpeed, 0.0)
        }

        runUntilFinish(arm.moveToAngular(thetaA = 133.degrees, thetaB = 0.degrees))
    }

    printToConsole { "Pitch < -10 deg (${navX.gyroscope.pitch.inUnit(degrees)}" }

    runParallelUntilOneFinishes {
        runUntil({ navX.gyroscope.pitch - 2.9.degrees > 10.degrees }, this@scoreTaxiBalance) {
            curvatureDrive(autoSpeed, 0.0)
        }

        runUntilFinish(arm.moveToAngular(thetaA = 133.degrees, thetaB = 0.degrees))
    }

    printToConsole { "Pitch > 10 deg (${navX.gyroscope.pitch.inUnit(degrees)}" }

    runUntil({ abs(navX.gyroscope.pitch-2.9.degrees) < 7.degrees }, this@scoreTaxiBalance) {
        curvatureDrive(autoSpeed, 0.0)
    }

    printToConsole { "abs(Pitch) < 10 deg (${navX.gyroscope.pitch.inUnit(degrees)}" }


    with(navX.gyroscope as HeadingProvider) {
        with(TurnPIDConstants(0.1, 0.0, 0.0)) {
            driveStraight(2.feet, 0.2)
            driveStraight((-1).feet, 0.2)
        }
    }

    runParallelUntilOneFinishes {
        runUntil({ abs(navX.gyroscope.pitch-2.9.degrees) > 10.0.degrees }, this@scoreTaxiBalance) {
            arcadeDrive(-autoSpeed, 0.0)
        }
        waitFor(3.seconds)
    }

    runForever(this@scoreTaxiBalance) {
        arcadeDrive(-0.675 * sin(navX.gyroscope.pitch-2.9.degrees),0.0)
    }
}