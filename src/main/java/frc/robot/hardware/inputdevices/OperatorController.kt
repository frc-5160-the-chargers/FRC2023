@file:Suppress("detekt.MagicNumber")
package frc.robot.hardware.inputdevices

import frc.chargers.hardware.inputdevices.ChargerController
import kotlin.math.abs

class OperatorController(port: Int) : ChargerController(port) {

    val conePresetButton = button(Button.kX)
    val cubePresetButton = button(Button.kY)
    val substationPresetButton = button(Button.kA)
    val restPresetButton = button(Button.kB)

    val coneLightButton = button(Button.kLeftBumper)
    val cubeLightButton = button(Button.kRightBumper)
    val intakePower: Double get() = listOf(rightTriggerAxis * 0.5, -leftTriggerAxis * 0.5).firstOrNull { abs(it) > 0.05 } ?: 0.0
}