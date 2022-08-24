@file:Suppress("detekt.MagicNumber")
package frc.robot.hardware.inputdevices

import frc.chargers.hardware.inputdevices.ChargerController

class OperatorController(port: Int) : ChargerController(port) {
    val outtake = button(Button.kA)
    val intake = button(Button.kB)
    val lowerArm = button(Button.kLeftBumper)
    val raiseArm = button(Button.kRightBumper)
    val toggleShooter = button(Button.kX)
    val runSerializerForward = button(Button.kY)
    val runSerializerReverse = button(Button.kLeftStick)
    val climberForward = button(Axis.kRightTrigger)
}