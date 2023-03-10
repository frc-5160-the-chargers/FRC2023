package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.value
import com.batterystaple.kmeasure.units.Degrees
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.robot.hardware.subsystems.Arm

class HoldArmAngular(private val arm: Arm, thetaA: Angle = arm.thetaA, thetaB: Angle = arm.thetaB) : CommandBase() {
    init {
        addRequirements(arm)
        name = "Hold Position"
    }

    private val jointAPID =
        UnitSuperPIDController(
            pidConstants = PIDConstants(4.0, 0.02, 0.0),
            getInput = { arm.thetaA },
            target = thetaA,
            outputRange = Scalar(-6.0)..Scalar(6.0),
        )

    private val jointBPID =
        UnitSuperPIDController(
            pidConstants = PIDConstants(2.0, 0.02, 0.0),
            getInput = { arm.thetaB },
            target = thetaB,
            outputRange = Scalar(-6.0)..Scalar(6.0),
        )

    override fun execute() {
        val outputA = -jointAPID.calculateOutput().value
        val outputB = -jointBPID.calculateOutput().value

//        val (feedForwardA, feedForwardB) = arm.calculateStaticPowers()
        arm.moveVoltages(Arm.JointVoltages(outputA /*+ feedForwardA*/, outputB /*+ feedForwardB*/))

        telemetry(outputA = outputA, outputB = outputB)
    }

    private fun telemetry(outputA: Double, outputB: Double) {
        SmartDashboard.putNumber("PID output A", outputA)
        SmartDashboard.putNumber("PID output B", outputB)

        SmartDashboard.putNumber("error A", jointAPID.error.inUnit(Degrees))
        SmartDashboard.putNumber("error B", jointBPID.error.inUnit(Degrees))
    }
}