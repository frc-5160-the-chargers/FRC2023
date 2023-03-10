@file:Suppress("detekt.MagicNumber")
package frc.robot.hardware.inputdevices

import frc.chargers.hardware.inputdevices.ChargerController
import kotlin.math.abs

class OperatorController(port: Int) : ChargerController(port) {

    val switchCommandButton = button(Button.kX)
    val coneButton = button(Button.kLeftBumper)
    val cubeButton = button(Button.kRightBumper)
    val intakePower: Double get() = listOf(rightTriggerAxis * 0.5, -leftTriggerAxis * 0.5).firstOrNull { abs(it) > 0.05 } ?: 0.0
}