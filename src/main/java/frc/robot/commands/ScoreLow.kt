package frc.robot.commands

import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.buildCommand
import frc.robot.hardware.subsystems.Arm
import frc.robot.hardware.subsystems.Intake

/**
 * A command that allows the arm to automatically score low; primarily used in auto.
 */
fun Arm.scoreLow(intake: Intake): Command = buildCommand{
    loopFor(0.1.seconds,this@scoreLow){
        moveVoltages(0.0,1.0)
    }

    loopFor(0.5.seconds, intake) {
        intake.setCustomPower(-0.25)
    }

    loopFor(0.06.seconds,this@scoreLow){
        moveVoltages(0.0,-1.0)
        intake.setCustomPower(0.0)
    }

    runOnce(this@scoreLow){
        moveSpeeds(0.0,0.0)
    }
}