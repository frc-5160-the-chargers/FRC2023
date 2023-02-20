package frc.robot.hardware.subsystems

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Shooter(
    private val frontMotor: MotorController,
    private val backMotor: MotorController,
    private val frontPower: Double,
    private val backPower: Double,
) : SubsystemBase() {
    private var state: State = State.DISABLED

    fun enable() {
        state = State.ENABLED
    }

    fun disable() {
        state =
            State.DISABLED
    }

    fun toggle() {
        state = when (state) {
            State.ENABLED -> State.DISABLED
            State.DISABLED -> State.ENABLED
        }
    }

    override fun periodic() {
        when(state) {
            State.ENABLED -> {
                frontMotor.set(frontPower)
                backMotor.set(backPower)
            }
            State.DISABLED -> {
                frontMotor.set(0.0)
                backMotor.set(0.0)
            }
        }
    }

    internal enum class State {
        ENABLED, DISABLED;
    }
}
