package frc.robot.commands.auto

import com.batterystaple.kmeasure.quantities.abs
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.sin
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.feet
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.commands.buildCommand
import frc.chargers.commands.drivetrainCommands.driveStraight
import frc.chargers.constants.TurnPIDConstants
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain


private const val autoSpeed = 0.4
fun EncoderDifferentialDrivetrain.taxiBalance(navX: NavX) = buildCommand {
    loopUntil({ navX.gyroscope.pitch-2.9.degrees < -10.degrees }, this@taxiBalance) {
        curvatureDrive(autoSpeed,0.0)
    }

    printToConsole { "Pitch < -10 deg (${navX.gyroscope.pitch.inUnit(degrees)}" }

    loopUntil({ navX.gyroscope.pitch-2.9.degrees > 10.degrees }, this@taxiBalance) {
        curvatureDrive(autoSpeed, 0.0)
    }

    printToConsole { "Pitch > 10 deg (${navX.gyroscope.pitch.inUnit(degrees)}" }

    loopUntil({ abs(navX.gyroscope.pitch-2.9.degrees) < 7.degrees }, this@taxiBalance) {
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
        loopUntil({ abs(navX.gyroscope.pitch-2.9.degrees) > 10.0.degrees }, this@taxiBalance) {
            arcadeDrive(-autoSpeed, 0.0)
        }
        waitFor(3.seconds)
    }

    +this@taxiBalance.balance(navX.gyroscope)
}