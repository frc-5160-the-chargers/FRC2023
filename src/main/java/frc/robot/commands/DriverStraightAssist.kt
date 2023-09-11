package frc.robot.commands

import com.batterystaple.kmeasure.dimensions.AngleDimension
import com.batterystaple.kmeasure.dimensions.ScalarDimension
import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.quantities.value
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.buildCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.DifferentialDrivetrain

context(HeadingProvider)
fun DifferentialDrivetrain.driverStraightAssistCommand(forwardPower: () -> Double, targetHeading: Angle): Command = buildCommand {
    val turnPID = UnitSuperPIDController<AngleDimension, ScalarDimension>(
        pidConstants = PIDConstants(0.5, 0.0, 0.0),
        getInput = { heading },
        target = heading
    )

    loopForever(this@driverStraightAssistCommand) {
        arcadeDrive(forwardPower(), turnPID.calculateOutput().value)
    }
}