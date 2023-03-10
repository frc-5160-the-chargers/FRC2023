package frc.robot.hardware.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.motorcontrol.rev.ChargerCANSparkMax
import frc.chargers.wpilibextensions.motorcontrol.speed

class Intake(
    private val motorOne: ChargerCANSparkMax,
    private val motorTwo: ChargerCANSparkMax,
    private val motorOneMultiplier: Double = 1.0,
    private val motorTwoMultiplier: Double = 1.0
) : SubsystemBase() {

    private var motorPower: Double = 0.0;

    fun setCustomPower(power: Double) {
        motorPower = power
    }

    fun forward() {
        motorPower = 0.5
    }

    fun disable() {
        motorPower = 0.0
    }

    fun reverse() {
        motorPower = -0.3
    }

    // this function is called repeatedly.
    override fun periodic() {
        motorOne.speed = motorPower*motorOneMultiplier
        motorTwo.speed = motorPower*motorTwoMultiplier

        telemetry()
    }

    private fun telemetry() {
        SmartDashboard.putNumber("Current A", motorOne.outputCurrent)
        SmartDashboard.putNumber("Current B", motorTwo.outputCurrent)
    }
}