package frc.robot.hardware.subsystems

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.utils.a
import frc.chargers.wpilibextensions.motorcontrol.speed
import frc.chargers.utils.math.equations.compileMultiline
import frc.chargers.utils.math.equations.stallTorqueToVoltage
import frc.chargers.utils.p
import org.ejml.data.DMatrixRMaj
import org.ejml.equation.Equation
import org.ejml.simple.SimpleMatrix
import kotlin.math.cos
import kotlin.math.sin
import org.ejml.data.DMatrixRMaj as M

/**
 * The Arm Subsystem of the robot.
 * Only current functionality is setting voltage to the arm motors,
 * as the lack of encoders prevents any advanced controls from happening.
 */
@Suppress("LongParameterList")
class Arm(
    private val proximalMotors: EncoderMotorController,
    private val distalMotor: EncoderMotorController,
    private val jointAEncoder: PositionEncoder = proximalMotors.encoder,
    private val jointBEncoder: PositionEncoder = distalMotor.encoder,
    private var jointAOffset: Angle,
    private var jointBOffset: Angle,
    private val gearRatioA: Double,
    private val gearRatioB: Double,
    val segmentALength: Distance,
    val segmentBLength: Distance
) : SubsystemBase() {



    private var softStopEnabled = true

    var q1: Angle get() = -jointAEncoder.angularPosition * gearRatioA + jointAOffset
        set(value) {
            jointAOffset = value - q1
        }
    var q2: Angle get() = -jointBEncoder.angularPosition * gearRatioB + jointBOffset
        set(value) {
            jointBOffset = value - q2
        }

    val thetaA: Angle get() = q1
    val thetaB: Angle get() = q1 + q2

    val forward: Distance get() = segmentALength * cos(thetaA) + segmentBLength * cos(thetaB)
    val up: Distance get() = segmentALength * sin(thetaA) + segmentBLength * sin(thetaB)


    fun moveVoltages(voltages: JointVoltages) {
        moveVoltages(voltages.jointAVoltage, voltages.jointBVoltage)
    }

    fun moveVoltages(jointAVoltage: Double, jointBVoltage: Double) {
        SmartDashboard.putNumber("Joint A Voltage (V)", jointAVoltage)
        SmartDashboard.putNumber("Joint B Voltage (V)", jointBVoltage)

        proximalMotors.setVoltage(jointAVoltage)
        distalMotor.setVoltage(jointBVoltage)
    }


    fun moveSpeeds(omegaA: Double = 0.0, omegaB: Double = 0.0) {
        proximalMotors.speed = omegaA
        distalMotor.speed = omegaB
    }

    fun moveCartesian(forward: Double, up: Double) {
        val thetaDot = calculateThetaDot(DMatrixRMaj(p[forward, up]))

        moveSpeeds(thetaDot[0], thetaDot[1])
    }


    override fun periodic() {
        telemetry()
    }

    private fun telemetry() {
        SmartDashboard.getNumber("Joint A Offset (º)", Double.NaN).takeIf { !it.isNaN() }?.ofUnit(degrees)?.let {
            jointAOffset = it
        }
        SmartDashboard.getNumber("Joint B Offset (º)", Double.NaN).takeIf { !it.isNaN() }?.ofUnit(degrees)?.let {
            jointBOffset = it
        }

        SmartDashboard.putNumber("Joint A Offset (º)", jointAOffset.inUnit(degrees))
        SmartDashboard.putNumber("Joint B Offset (º)", jointBOffset.inUnit(degrees))

        SmartDashboard.putNumber("Theta A (º)", thetaA.inUnit(degrees))
        SmartDashboard.putNumber("Theta B (º)", thetaB.inUnit(degrees))
        SmartDashboard.putNumber("Q1 (º)", q1.inUnit(degrees))
        SmartDashboard.putNumber("Q2 (º)", q2.inUnit(degrees))
        SmartDashboard.putBoolean("Soft Stop Enabled", softStopEnabled)

    }

    @Suppress("VariableNaming", "Unused")
    fun calculateThetaDot(v: DMatrixRMaj): DMatrixRMaj {
        val thetaA = thetaA.inUnit(radians)
        val thetaB = thetaB.inUnit(radians)
        val lA = segmentALength.inUnit(meters)
        val lB = segmentBLength.inUnit(meters)

        val k = sin(thetaA - thetaB)

        val J = DMatrixRMaj(a[
            p[-cos(thetaB) /(lA * k),  sin(thetaB) /(lA * k)],
            p[ cos(thetaA) /(lB * k), -sin(thetaA) /(lB * k)]
        ])


        val theta_dot = SimpleMatrix(J).mult(SimpleMatrix(v))

        return theta_dot.getMatrix()
    }

    @Suppress("VariableNaming", "Unused", "NonAsciiCharacters")
    private fun calculateStaticPowers(): JointVoltages {
        val l1 = segmentALength.inUnit(meters)
        val l2 = segmentBLength.inUnit(meters)
        val mα = 1.2
        val mβ = 0.75
        val mψ = 1.9

        gravityCompEquation.alias(
            l1, "l1", l2, "l2",
            mα, "mα", mβ, "mβ", mψ, "mψ",
            q1.inUnit(radians), "q1", q2.inUnit(radians), "q2"

        )
        compiledGravityCompSequence.perform()

        val torque = gravityCompEquation.lookupDDRM("τ")
        val torqueA = torque[0]
        val torqueB = torque[1]

        val gearedTorqueA = torqueA * gearRatioA
        val gearedTorqueB = torqueB * gearRatioB

        val voltageA = stallTorqueToVoltage(gearedTorqueA.ofUnit(newtons*meters))
        val voltageB = stallTorqueToVoltage(gearedTorqueB.ofUnit(newtons*meters))

        return JointVoltages(voltageA.inUnit(volts), voltageB.inUnit(volts))
    }

    data class JointVoltages(val jointAVoltage: Double, val jointBVoltage: Double)


    private val gravityCompEquation = Equation().apply {
        process("macro crossMatrix( v ) = ( [0, -v(2), v(1); v(2), 0, -v(0); -v(1), v(0), 0] )")
        val iHat = M(p[1.0, 0.0, 0.0])
        val jHat = M(p[0.0, 1.0, 0.0])
        val kHat = M(p[0.0, 0.0, 1.0])

        alias(iHat, "iHat", jHat, "jHat", kHat, "kHat")
        alias(0.0, "l1", 0.0, "l2", 0.0, "mα", 0.0, "mβ", 0.0, "mψ", 0.0, "q1", 0.0, "q2")
    }

    private val compiledGravityCompSequence = gravityCompEquation.compileMultiline(
        """
        g = [0, -9.81, 0]'
        q = [q1, q2]'

        J_0AR = [kHat [0;0;0]]
        J_ABR = [[0;0;0] kHat]
        J_0BR = J_0AR + J_ABR

        C_A0 = ( [cos(q(0)), -sin(q(0)), 0; sin(q(0)), cos(q(0)), 0; 0, 0, 0] )
        C_BA = ( [cos(q(1)), -sin(q(1)), 0; sin(q(1)), cos(q(1)), 0; 0, 0, 0] )

        r_oa_A = [l1   0 0]'
        r_oα_A = [l1/2 0 0]'
        r_aβ_B = [l2/2 0 0]'
        r_aψ_B = [l2   0 0]'

        r_oa_0 = C_A0 * r_oa_A
        r_oα_0 = C_A0 * r_oα_A
        r_aβ_0 = C_A0 * C_BA * r_aβ_B
        r_aψ_0 = C_A0 * C_BA * r_aψ_B

        J_0AD = -crossMatrix(r_oa_0) * J_0AR

        J_0αD = -crossMatrix(r_oα_0) * J_0AR
        J_0βD = J_0AD - crossMatrix(r_aβ_0) * J_0BR
        J_0ψD = J_0AD - crossMatrix(r_aψ_0) * J_0BR

        F_gα = mα * g
        F_gβ = mβ * g
        F_gψ = mψ * g

        J_θ = [1 0; -1 1]

        a = J_0αD' * F_gα
        b = J_0βD' * F_gβ
        c = J_0ψD' * F_gψ

        τ = -J_θ' * (J_0αD' * F_gα + J_0βD' * F_gβ + J_0ψD' * F_gψ)

        """.trimIndent()
    )
}

