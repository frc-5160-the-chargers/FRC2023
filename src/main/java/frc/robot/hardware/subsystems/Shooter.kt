package frc.robot.hardware.subsystems

import frc.robot.hardware.subsystems.Shooter.State.ENABLED
import frc.robot.hardware.subsystems.Shooter.State.DISABLED
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Shooter(
    private val frontMotor: MotorController,
    private val backMotor: MotorController,
    private val frontPower: Double,
    private val backPower: Double,
) : SubsystemBase() {
    private var state: State = DISABLED

    fun enable() {
        state = ENABLED
    }

    fun disable() {
        state = DISABLED
    }

    fun toggle() {
        state = when (state) {
            ENABLED -> DISABLED
            DISABLED -> ENABLED
        }
    }

    override fun periodic() {
        when(state) {
            ENABLED -> {
                frontMotor.set(frontPower)
                backMotor.set(backPower)
            }
            DISABLED -> {
                frontMotor.set(0.0)
                backMotor.set(0.0)
            }
        }
    }

    internal enum class State {
        ENABLED, DISABLED;
    }
}
