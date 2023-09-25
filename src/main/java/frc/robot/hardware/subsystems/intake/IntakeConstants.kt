package frc.robot.hardware.subsystems.intake

import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax


object IntakeMotors{
    val left: EncoderMotorController = neoSparkMax(6)
    val right: EncoderMotorController = neoSparkMax(6)
}