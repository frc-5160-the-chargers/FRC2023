package frc.robot.hardware.subsystems.arm

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.volts
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger


data class JointVoltages(
    val proximalV: Voltage,
    val distalV: Voltage
)


class Arm(
    private val io: ArmIO
): SubsystemBase() {
    private val inputs = ArmIO.Inputs()

    private var proximalJointOffset: Angle = ArmConstants.PROXIMAL_JOINT_OFFSET
    private var distalJointOffset: Angle = ArmConstants.DISTAL_JOINT_OFFSET

    var thetaProximal: Angle
        get() = -inputs.thetaProximal * ArmConstants.GEAR_RATIO_PROXIMAL + proximalJointOffset
        set(value) {
            proximalJointOffset = value - thetaProximal
        }
    var thetaDistal: Angle
        get() = -inputs.thetaDistal * ArmConstants.GEAR_RATIO_DISTAL + distalJointOffset
        set(value) {
            distalJointOffset = value - thetaDistal
        }

    fun moveVoltages(proximalV: Voltage, distalV: Voltage){
        io.setVoltages(proximalV,distalV)
    }

    fun moveVoltages(voltages: JointVoltages){
        io.setVoltages(voltages.proximalV,voltages.distalV)
    }

    fun movePercentOut(proximalOutput: Double, distalOutput: Double){
        io.setVoltages(proximalOutput * 12.volts, distalOutput * 12.volts)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.getInstance().processInputs("Arm",inputs)
    }

}