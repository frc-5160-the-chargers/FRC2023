package frc.robot.commands.auto

import com.batterystaple.kmeasure.quantities.sin
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.seconds
import frc.chargers.commands.buildCommand
import frc.chargers.hardware.sensors.gyroscopes.ThreeAxisGyroscope
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain

/**
 * The balance command for the robot. Uses the buildCommand DSL,
 * returned from an extension function of the EncoderDifferentialDrivetrain class.
 */
fun EncoderDifferentialDrivetrain.balance(gyro: ThreeAxisGyroscope) = buildCommand{
    loopFor(5.seconds,this@balance) {
        arcadeDrive(-0.675 * sin(gyro.pitch-2.9.degrees),0.0)
    }

    loopForever(this@balance) {
        arcadeDrive(-0.2 * sin(gyro.pitch-2.9.degrees),0.0)
    }
}