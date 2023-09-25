package frc.robot.hardware.subsystems.intake

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake(
    private val io: IntakeIO
): SubsystemBase(){
    private val inputs = IntakeIO.Inputs()

    val leftSpeed: AngularVelocity
        get() = inputs.leftSpeed
    val rightSpeed: AngularVelocity
        get() = inputs.rightSpeed

    fun setVoltage(outputVolts: Voltage){
        io.setVoltage(outputVolts)
    }

    fun setSpeed(percentOut: Double){
        io.setVoltage(percentOut * 12.volts)
    }

    override fun periodic(){
        io.updateInputs(inputs)
    }


}