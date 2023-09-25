package frc.robot.hardware.subsystems.arm

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.chargers.advantagekitextensions.ChargerLoggableInputs
import frc.chargers.wpilibextensions.motorcontrol.setVoltage


object ArmIOReal: ArmIO{
    override fun setVoltages(proximalV: Voltage, distalV: Voltage) {
        ArmMotors.proximal.setVoltage(proximalV)
        ArmMotors.distal.setVoltage(distalV)
    }

    override fun updateInputs(inputs: ArmIO.Inputs) {
        inputs.apply{
            thetaProximal = ArmMotors.proximal.encoder.angularPosition
            thetaDistal = ArmMotors.distal.encoder.angularPosition
            proximalJointVelocity = ArmMotors.proximal.encoder.angularVelocity
            distalJointVelocity = ArmMotors.distal.encoder.angularVelocity
        }
    }
}


object ArmIOSim: ArmIO{

    private val proximalJointSim: SingleJointedArmSim = SingleJointedArmSim(
        DCMotor.getNEO(2),
        ArmConstants.GEAR_RATIO_PROXIMAL,
        SingleJointedArmSim.estimateMOI(
            ArmConstants.PROXIMAL_SEGMENT_LENGTH.inUnit(meters),
            ArmConstants.ARM_PROXIMAL_MASS.inUnit(kilo.grams)
        ),
        ArmConstants.PROXIMAL_SEGMENT_LENGTH.inUnit(meters),
        0.0,
        360.0,
        true
    )

    private val distalJointSim: SingleJointedArmSim = SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        ArmConstants.GEAR_RATIO_DISTAL,
        SingleJointedArmSim.estimateMOI(
            ArmConstants.DISTAL_SEGMENT_LENGTH.inUnit(meters),
            ArmConstants.ARM_DISTAL_MASS.inUnit(kilo.grams)
        ),
        ArmConstants.DISTAL_SEGMENT_LENGTH.inUnit(meters),
        0.0,
        360.0,
        true
    )
    override fun setVoltages(proximalV: Voltage, distalV: Voltage) {
        proximalJointSim.setInputVoltage(proximalV.inUnit(volts))
        distalJointSim.setInputVoltage(distalV.inUnit(volts))
    }

    override fun updateInputs(inputs: ArmIO.Inputs) {
        inputs.apply{
            thetaProximal = proximalJointSim.angleRads.ofUnit(radians)
            thetaDistal = distalJointSim.angleRads.ofUnit(radians)
            proximalJointVelocity = proximalJointSim.velocityRadPerSec.ofUnit(radians/seconds)
            distalJointVelocity = distalJointSim.velocityRadPerSec.ofUnit(radians/seconds)
        }
    }

}


interface ArmIO {
    class Inputs: ChargerLoggableInputs(){
        var thetaProximal by loggedQuantity(0.degrees,"proximalAngleDegrees", degrees)
        var thetaDistal by loggedQuantity(0.degrees,"distalAngleDegrees", degrees)

        var proximalJointVelocity by loggedQuantity(0.degrees/0.seconds,"proximalJointVelocityDegPerSec",degrees/seconds)
        var distalJointVelocity by loggedQuantity(0.degrees/0.seconds,"distalJointVelocityDegPerSec",degrees/seconds)
    }

    fun setVoltages(proximalV: Voltage, distalV: Voltage)
    fun updateInputs(inputs: Inputs)
}