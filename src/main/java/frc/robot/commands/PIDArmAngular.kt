package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.value
import com.batterystaple.kmeasure.units.degrees
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.robot.hardware.subsystems.Arm

fun Arm.holdAngular(): PIDArmAngular = PIDArmAngular(
    this,
    thetaA,
    thetaB,
    jointAPIDConstants = PIDConstants(15.0, 0.02, 1.0),
    jointBPIDConstants = PIDConstants(2.5, 0.01, 0.35),
    jointAOutputRange = Scalar(-8.0)..Scalar(8.0),
    jointBOutputRange = Scalar(-6.0)..Scalar(6.0),
)

fun Arm.moveToAngular(thetaA: Angle, thetaB: Angle): PIDArmAngular = PIDArmAngular(
    this,
    thetaA,
    thetaB,
    jointAPIDConstants = PIDConstants(20.0, 0.02, 1.0),
    jointBPIDConstants = PIDConstants(12.0, 0.01, 0.7),
    jointAOutputRange = Scalar(-12.0)..Scalar(12.0),
    jointBOutputRange = Scalar(-10.0)..Scalar(10.0),
)

class PIDArmAngular(
    private val arm: Arm,
    thetaA: Angle = arm.thetaA,
    thetaB: Angle = arm.thetaB,
    jointAPIDConstants: PIDConstants,
    jointBPIDConstants: PIDConstants,
    jointAOutputRange: ClosedRange<Scalar>,
    jointBOutputRange: ClosedRange<Scalar>
) : CommandBase() {
    init {
        addRequirements(arm)
        name = "Hold Position Angular"
    }

    private val jointAPID =
        UnitSuperPIDController(
            pidConstants = jointAPIDConstants,
            getInput = { arm.thetaA },
            target = thetaA,
            outputRange = jointAOutputRange,
        )

    private val jointBPID =
        UnitSuperPIDController(
            pidConstants = jointBPIDConstants,
            getInput = { arm.thetaB },
            target = thetaB,
            outputRange = jointBOutputRange,
        )

    var thetaA: Angle by jointAPID::target
    var thetaB: Angle by jointBPID::target

    override fun execute() {
        val outputA = -jointAPID.calculateOutput().value
        val outputB = -jointBPID.calculateOutput().value
//        val outputB = 0.0

//        val (feedForwardA, feedForwardB) = arm.calculateStaticPowers()
        arm.moveVoltages(Arm.JointVoltages(outputA /*+ feedForwardA*/, outputB /*+ feedForwardB*/))

        telemetry(outputA = outputA, outputB = outputB)
    }

    private fun telemetry(outputA: Double, outputB: Double) {
        SmartDashboard.putNumber("PID output A", outputA)
        SmartDashboard.putNumber("PID output B", outputB)

        SmartDashboard.putNumber("error A (º)", jointAPID.error.inUnit(degrees))
        SmartDashboard.putNumber("error B (º)", jointBPID.error.inUnit(degrees))
    }

    companion object {
//        fun fromCartesian(
//            arm: Arm,
//            forward: Distance = arm.forward,
//            up: Distance = arm.up,
//            jointAPIDConstants: PIDConstants,
//            jointBPIDConstants: PIDConstants,
//            jointAOutputRange: ClosedRange<Scalar>,
//            jointBOutputRange: ClosedRange<Scalar>
//        ) {
//            val segmentASquared = arm.segmentALength * arm.segmentALength
//            val segmentBSquared = arm.segmentBLength * arm.segmentBLength
//            val distanceSquared = forward * forward + up * up
//
//            val thetaBRadianTriangularAngle = arccos((segmentASquared + segmentASquared - distanceSquared) / (2*arm.segmentALength*segmentBSquared))
//
//        }
    }
}