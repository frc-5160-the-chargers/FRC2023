package frc.robot.hardware.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.motorcontrol.rev.ChargerCANSparkMax
import frc.chargers.wpilibextensions.motorcontrol.speed

class Intake(
    public val motorOne: ChargerCANSparkMax,
    public val motorTwo: ChargerCANSparkMax,
    private val motorOneMultiplier: Double = 1.0,
    private val motorTwoMultiplier: Double = 1.0,
    private val passiveSpeed: Double = 0.0
) : SubsystemBase() {

    var passiveSpeedEnabled: Boolean = false

    private var motorPower: Double = 0.0;

    fun setCustomPower(power: Double) {
        motorPower = power
    }

    fun intake() {
        motorPower = 0.5
    }

    fun disable() {
        motorPower = 0.0
    }

    fun outtake() {
        motorPower = -0.2
    }

    // this function is called repeatedly.
    override fun periodic() {
        motorOne.speed = motorPower*motorOneMultiplier
        motorTwo.speed = motorPower*motorTwoMultiplier

        /*
        if (passiveSpeedEnabled) {
            if(motorOne.speed == 0.0) motorOne.speed = passiveSpeed
            if(motorTwo.speed == 0.0) motorTwo.speed = passiveSpeed
        }
         */



        telemetry()
    }

    private fun telemetry() {
        SmartDashboard.putNumber("Current A", motorOne.outputCurrent)
        SmartDashboard.putNumber("Current B", motorTwo.outputCurrent)
    }
}