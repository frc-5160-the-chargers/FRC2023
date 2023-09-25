@file:Suppress("detekt.MagicNumber")
package frc.robot.hardware.inputdevices

import com.batterystaple.kmeasure.quantities.times
import com.batterystaple.kmeasure.units.volts
import frc.chargers.hardware.inputdevices.ChargerController
import frc.robot.hardware.subsystems.arm.JointVoltages
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

class OperatorController(port: Int) : ChargerController(port) {


    val intakePower: Double get() =
        listOf(rightTriggerAxis * 0.75, -leftTriggerAxis * 0.45)
            .firstOrNull { abs(it) > 0.05 } ?: 0.0

    private fun armPowerCurve(x: Double) = sign(x) * x.pow(2)
    // proximal
    val jointAPower get() = -armPowerCurve(leftY)
    // distal
    val jointBPower get() = 0.75 * armPowerCurve(rightY)

    val armVoltages: JointVoltages
        get() = JointVoltages(
        proximalV = jointAPower * 10.0 * 0.7.volts,
        distalV = jointBPower * 10.0 * 0.8.volts
    )

}