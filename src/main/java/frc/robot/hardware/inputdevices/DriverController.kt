@file:Suppress("detekt.MagicNumber")
package frc.robot.hardware.inputdevices

import frc.chargers.hardware.inputdevices.ChargerController
import frc.chargers.utils.math.mapBetweenRanges
import frc.chargers.wpilibextensions.kinematics.ChassisPowers

class DriverController(
    port: Int,
    deadband: Double,
    private val shouldInvertStraightDriveDirection: Boolean = false,
    private val shouldInvertRotationDirection: Boolean = false,
    private val turboModeMultiplierRange: ClosedRange<Double> = 0.0..1.0,
) : ChargerController(port, deadband) {
    val lowerLeftArmButton = button(Axis.kLeftTrigger, threshold = 0.5)
    val raiseLeftArmButton = button(Button.kLeftBumper)
    val lowerRightArmButton = button(Axis.kRightTrigger, threshold = 0.5)
    val raiseRightArmButton = button(Button.kRightBumper)

    val curvatureOutput: ChassisPowers get() {
        var forwardsPower = leftY.withDeadband()
        var rotationPower = rightX.withDeadband()

        if (shouldInvertStraightDriveDirection) { forwardsPower *= -1 }
        if (shouldInvertRotationDirection) { rotationPower *= -1 }

        return ChassisPowers(
            xPower = forwardsPower * 0.35,
            rotationPower = rotationPower * -0.4
        )
    }

    val turboModePower: Double get() =
        rightTriggerAxis
            .mapBetweenRanges(from = 0.0..1.0, to = turboModeMultiplierRange)
}