package frc.robot.hardware.subsystems.intake

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.chargers.advantagekitextensions.ChargerLoggableInputs
import frc.chargers.wpilibextensions.motorcontrol.setVoltage

object IntakeIOReal: IntakeIO{
    override fun setVoltage(outputVolts: Voltage) {
        IntakeMotors.left.setVoltage(outputVolts)
        IntakeMotors.right.setVoltage(-outputVolts)
    }

    override fun updateInputs(inputs: IntakeIO.Inputs) {
        inputs.apply{
            leftSpeed = IntakeMotors.left.encoder.angularVelocity
            rightSpeed = IntakeMotors.right.encoder.angularVelocity
        }
    }

}

object IntakeIOSim: IntakeIO{
    val leftMotorSim: FlywheelSim = FlywheelSim(
        DCMotor.getNEO(1),
        1.0,
        0.004
    )
    val rightMotorSim: FlywheelSim = FlywheelSim(
        DCMotor.getNEO(1),
        1.0,
        0.004
    )

    override fun setVoltage(outputVolts: Voltage) {
        leftMotorSim.setInputVoltage(outputVolts.inUnit(volts))
        rightMotorSim.setInputVoltage(-outputVolts.inUnit(volts))
    }

    override fun updateInputs(inputs: IntakeIO.Inputs) {
        inputs.apply{
            leftSpeed = leftMotorSim.angularVelocityRadPerSec.ofUnit(radians/seconds)
            rightSpeed = rightMotorSim.angularVelocityRadPerSec.ofUnit(radians/seconds)
        }
    }


}

interface IntakeIO{
    class Inputs: ChargerLoggableInputs(){
        var leftSpeed by loggedQuantity(
            logUnit = degrees/seconds,
            "leftVelocityDegPerSec"
        )
        var rightSpeed by loggedQuantity(
            logUnit = degrees/seconds,
            "rightVelocityDegPerSec"
        )
    }

    fun setVoltage(outputVolts: Voltage)
    fun updateInputs(inputs: Inputs)

}