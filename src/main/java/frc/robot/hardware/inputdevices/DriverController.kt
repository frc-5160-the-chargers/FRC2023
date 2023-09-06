@file:Suppress("detekt.MagicNumber")
package frc.robot.hardware.inputdevices

import frc.chargers.hardware.inputdevices.ChargerController
import frc.chargers.utils.math.mapBetweenRanges
import frc.chargers.wpilibextensions.kinematics.ChassisPowers

class DriverController(
    port: Int,
    deadband: Double,
    private val forwardsPowerScale: Double,
    private val rotationPowerScale: Double,
    private val shouldInvertStraightDriveDirection: Boolean = false,
    private val shouldInvertRotationDirection: Boolean = false,
    private val turboModeMultiplierRange: ClosedRange<Double>,
    private val precisionModeDividerRange: ClosedRange<Double>,
) : ChargerController(port, deadband) {
    val switchCameraButton = button(Button.kX)

    val curvatureOutput: ChassisPowers get() {
        var forwardsPower = leftY.withDeadband()
        var rotationPower = rightX.withDeadband()

        if (shouldInvertStraightDriveDirection) { forwardsPower *= -1 }
        if (shouldInvertRotationDirection) { rotationPower *= -1 }

        return ChassisPowers(
            xPower = forwardsPower * forwardsPowerScale * turboModeMultiplier * precisionModeMultiplier,
            rotationPower = -rotationPower * rotationPowerScale * turboModeMultiplier * precisionModeMultiplier
        )
    }

    val driverStraightAssistEnabled: Boolean get() = leftY in -0.2..0.2

    private val turboModeMultiplier: Double get() =
        leftTriggerAxis
            .mapBetweenRanges(from = 0.0..1.0, to = turboModeMultiplierRange)

    private val precisionModeMultiplier: Double get() =
        1 / rightTriggerAxis
            .mapBetweenRanges(from = 0.0..1.0, to = precisionModeDividerRange)
}