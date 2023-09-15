package frc.robot.commands

import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.Scalar
import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.value
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.robot.hardware.subsystems.Arm
/*
class HoldArmCartesian(
    private val arm: Arm,
    private val forward: Distance = arm.forward,
    private val up: Distance = arm.up,
    private val upTolerance: Distance? = null,
    private val forwardTolerance: Distance? = null
) : CommandBase() {
    init {
        addRequirements(arm)
        name = "Hold Position Cartesian"
    }

    private val forwardPID =
        UnitSuperPIDController(
            pidConstants = PIDConstants(0.5, 0.00, 0.0),
            getInput = { arm.forward },
            target = forward,
            outputRange = Scalar(-6.0)..Scalar(6.0),
        )

    private val upPID =
        UnitSuperPIDController(
            pidConstants = PIDConstants(0.5, 0.02, 0.0),
            getInput = { arm.up },
            target = up,
            outputRange = Scalar(-6.0)..Scalar(6.0),
        )

    override fun execute() {
        val outputForward = forwardPID.calculateOutput().value
        val outputUp = upPID.calculateOutput().value

        arm.moveCartesian(forward = outputForward, up = outputUp)

        telemetry(outputForward = outputForward, outputUp = outputUp)
    }

    override fun isFinished(): Boolean {
        if (upTolerance == null) return false
        if (forwardTolerance == null) return false

        return (arm.forward in (forward - forwardTolerance)..(forward + forwardTolerance)) &&
                (arm.up in (up - upTolerance)..(up + upTolerance))
    }


    private fun telemetry(outputForward: Double, outputUp: Double) {
        SmartDashboard.putNumber("PID output Up", outputForward)
        SmartDashboard.putNumber("PID output Forward", outputUp)

        SmartDashboard.putNumber("error A (m)", forwardPID.error.inUnit(meters))
        SmartDashboard.putNumber("error B (m)", upPID.error.inUnit(meters))
    }
}
 */