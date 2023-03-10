package frc.robot.hardware.subsystems

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.Degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.chargers.hardware.motorcontrol.EncoderMotorController
import frc.chargers.hardware.sensors.encoders.PositionEncoder
import frc.chargers.wpilibextensions.motorcontrol.speed
import frc.robot.math.cos
import frc.robot.math.matrix.a
import frc.robot.math.matrix.compileMultiline
import frc.robot.math.sin
import frc.robot.math.stallTorqueNmToVoltage
import org.ejml.equation.Equation
import org.ejml.data.DMatrixRMaj as M

class Arm(
    private val jointAMotors: EncoderMotorController,
    private val jointBMotor: EncoderMotorController,
    private val swivelMotor: CANSparkMax, // TODO: Add current stuff to ChargerLib and make type some interface
    jointAEncoder: PositionEncoder/*? = null*/,
    jointBEncoder: PositionEncoder/*? = null*/,
    private var jointAOffset: Angle,
    private var jointBOffset: Angle,
    private val gearRatioA: Double, // TODO: Move to own encoder
    private val gearRatioB: Double,
    val segmentALength: Distance,
    val segmentBLength: Distance,
    private val jointSpeedMultiplier: Double = 1.0,
) : SubsystemBase() { // TODO: motor encoder offsets
    val jointAEncoder: PositionEncoder = jointAEncoder/* ?: jointAMotors.encoder*/
    val jointBEncoder: PositionEncoder = jointBEncoder/* ?: jointBMotor.encoder*/
    private val encoderMultiplierA = if (jointAEncoder == null) gearRatioA else 1.0
    private val encoderMultiplierB = if (jointBEncoder == null) gearRatioB else 1.0

    private val q1: Angle get() = jointAEncoder.angularPosition * encoderMultiplierA + jointAOffset
    private val q2: Angle get() = jointBEncoder.angularPosition * encoderMultiplierB + jointBOffset
//    private val qDot1: AngularVelocity get() = jointAMotors.encoder.angularVelocity * gearRatioA
//    private val qDot2: AngularVelocity get() = jointBMotor.encoder.angularVelocity * gearRatioB

    val thetaA: Angle get() = q1
    val thetaB: Angle get() = q1 + q2
//    val omegaA: AngularVelocity get() = qDot1
//    val omegaB: AngularVelocity get() = qDot1 + qDot2

    val forward: Distance get() = segmentALength * cos(thetaA) + segmentBLength * cos(thetaB)
    val up: Distance get() = segmentALength * sin(thetaA) + segmentBLength * sin(thetaB)

    fun moveVoltages(voltages: JointVoltages) {
        jointAMotors.setVoltage(voltages.jointAVoltage)
        jointBMotor.setVoltage(voltages.jointBVoltage)
    }

    fun moveAngles(omegaA: Double = 0.0, omegaB: Double = 0.0, rotation: Double = 0.0) {
        jointAMotors.speed = omegaA
        jointBMotor.speed = omegaB

        rotate(rotation)
    }

    fun rotate(rotation: Double) {
        if (swivelMotor.outputCurrent > 1) {
            swivelMotor.speed = 0.0
        }

        swivelMotor.speed = rotation
    }

    override fun periodic() {
        telemetry()
    }

    private fun telemetry() {
        SmartDashboard.getNumber("Joint A Offset (º)", Double.NaN).takeIf { !it.isNaN() }?.ofUnit(Degrees)?.let {
            jointAOffset = it
        }
        SmartDashboard.getNumber("Joint B Offset (º)", Double.NaN).takeIf { !it.isNaN() }?.ofUnit(Degrees)?.let {
            jointBOffset = it
        }
        SmartDashboard.putNumber("Joint A Offset (º)", jointAOffset.inUnit(Degrees))
        SmartDashboard.putNumber("Joint B Offset (º)", jointBOffset.inUnit(Degrees))

        SmartDashboard.putNumber("Theta A (º)", thetaA.inUnit(Degrees))
        SmartDashboard.putNumber("Theta B (º)", thetaB.inUnit(Degrees))
//        SmartDashboard.putNumber("Omega A (º/s)", omegaA.inUnit(Degrees / seconds))
//        SmartDashboard.putNumber("Omega B (º/s)", omegaB.inUnit(Degrees / seconds))

        SmartDashboard.putNumber("Turret Current (A prob)", swivelMotor.outputCurrent)

    }

    fun calculateStaticPowers(): JointVoltages {
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

        val torque = gravityCompEquation.lookupDDRM("τ") // TODO: destructuring declaration
        val torqueA = torque[0]
        val torqueB = torque[1]

        val gearedTorqueA = torqueA * gearRatioA
        val gearedTorqueB = torqueB * gearRatioB

        val voltageA = stallTorqueNmToVoltage(gearedTorqueA)
//        val voltageA = stallTorqueNmToVoltage(gearedTorqueA/2) // TODO: for when two motors mounted
        val voltageB = stallTorqueNmToVoltage(gearedTorqueB)

        return JointVoltages(voltageA, voltageB)
    }

    data class JointVoltages(val jointAVoltage: Double, val jointBVoltage: Double)

    private val gravityCompEquation = Equation().apply {
        process("macro crossMatrix( v ) = ( [0, -v(2), v(1); v(2), 0, -v(0); -v(1), v(0), 0] )")
        val iHat = M(a[1.0, 0.0, 0.0])
        val jHat = M(a[0.0, 1.0, 0.0])
        val kHat = M(a[0.0, 0.0, 1.0])

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

