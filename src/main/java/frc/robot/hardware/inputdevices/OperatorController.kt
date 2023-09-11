@file:Suppress("detekt.MagicNumber")
package frc.robot.hardware.inputdevices

import frc.chargers.hardware.inputdevices.ChargerController
import frc.robot.hardware.subsystems.Arm
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

class OperatorController(port: Int) : ChargerController(port) {


    val intakePower: Double get() =
        listOf(rightTriggerAxis * 0.5, -leftTriggerAxis * 0.3)
            .firstOrNull { abs(it) > 0.05 } ?: 0.0

    private fun armPowerCurve(x: Double) = sign(x) * x.pow(2)
    val jointAPower get() = -armPowerCurve(leftY)
    val jointBPower get() = armPowerCurve(rightY)

    val armVoltages: Arm.JointVoltages get() = Arm.JointVoltages(
        jointAVoltage = jointAPower * 10.0,
        jointBVoltage = jointBPower * 10.0
    )
}