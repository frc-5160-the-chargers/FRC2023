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

/*
class HoldArmAngular(private val arm: Arm, thetaA: Angle = arm.thetaA, thetaB: Angle = arm.thetaB) : CommandBase() {
    init {
        addRequirements(arm)
        name = "Hold Position Angular"
    }

    val jointAPID =
        UnitSuperPIDController(
            pidConstants = PIDConstants(30.0, 0.02, 2.0),
            getInput = { arm.thetaA },
            target = thetaA,
            outputRange = Scalar(-12.0)..Scalar(12.0),
        )

    val jointBPID =
        UnitSuperPIDController(
            pidConstants = PIDConstants(15.0, 0.01, 0.7),
            getInput = { arm.thetaB },
            target = thetaB,
            outputRange = Scalar(-8.0)..Scalar(8.0),
        )

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
}

 */