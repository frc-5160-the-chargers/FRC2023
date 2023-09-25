package frc.robot.hardware.subsystems.arm

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Length
import com.batterystaple.kmeasure.quantities.Mass
import com.batterystaple.kmeasure.units.grams
import com.batterystaple.kmeasure.units.kilo
import com.revrobotics.CANSparkMax
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.ctre.falcon
import frc.chargers.hardware.motorcontrol.rev.SparkMaxConfiguration
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax

object ArmMotors{
    val proximal: EncoderMotorController = EncoderMotorControllerGroup(
        neoSparkMax(12),
        neoSparkMax(23),
        configuration = SparkMaxConfiguration(
            idleMode = CANSparkMax.IdleMode.kBrake
        )
    )
    val distal: EncoderMotorController = falcon(7)
}

object ArmConstants {
    const val GEAR_RATIO_PROXIMAL: Double = 0.0
    const val GEAR_RATIO_DISTAL: Double = 0.0

    val PROXIMAL_JOINT_OFFSET: Angle = Angle(0.0)
    val DISTAL_JOINT_OFFSET: Angle = Angle(0.0)

    val PROXIMAL_SEGMENT_LENGTH: Length = Length(0.0)
    val DISTAL_SEGMENT_LENGTH: Length = Length(0.0)

    // unverified numbers; sim won't be used much for the arm
    val ARM_PROXIMAL_MASS: Mass = 10.0.kilo.grams
    val ARM_DISTAL_MASS: Mass = 4.0.kilo.grams
}
