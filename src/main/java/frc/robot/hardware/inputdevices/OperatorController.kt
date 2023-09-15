@file:Suppress("detekt.MagicNumber")
package frc.robot.hardware.inputdevices

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.chargers.hardware.inputdevices.ChargerController
import frc.robot.hardware.subsystems.Arm
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

class OperatorController(port: Int) : ChargerController(port) {


    val intakePower: Double get() =
        listOf(rightTriggerAxis * 0.75, -leftTriggerAxis * 0.45)
            .firstOrNull { abs(it) > 0.05 } ?: 0.0

    private fun armPowerCurve(x: Double) = sign(x) * x.pow(2)
    val jointAPower get() = -armPowerCurve(leftY)
    val jointBPower get() = armPowerCurve(rightY)

    val armVoltages: Arm.JointVoltages get() = Arm.JointVoltages(
        jointAVoltage = jointAPower * 10.0,
        jointBVoltage = jointBPower * 10.0
    ).also{
        SmartDashboard.putNumber("Joint A desired volts",jointAPower * 10.0)
        SmartDashboard.putNumber("Joint B desired volts",jointBPower * 10.0)
    }
}