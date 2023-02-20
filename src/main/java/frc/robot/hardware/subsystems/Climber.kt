package frc.robot.hardware.subsystems

import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Climber(
    private val leftMotor: MotorController,
    private val rightMotor: MotorController,
    private val upPower: Double,
    private val downPower: Double
) : SubsystemBase() {
    private var leftState: State = State.STEADY_STATE
    private var rightState: State = State.STEADY_STATE

    fun climbLeft() {
        leftState = State.CLIMBING
    }

    fun climbRight() {
        rightState = State.CLIMBING
    }

    fun lowerLeft() {
        leftState = State.LOWERING
    }

    fun lowerRight() {
        rightState = State.LOWERING
    }

    fun climb() {
        climbLeft()
        climbRight()
    }

    fun lower() {
        lowerLeft()
        lowerRight()
    }

    fun stop() {
        leftState = State.STEADY_STATE
        rightState = State.STEADY_STATE
    }

    override fun periodic() {
        when(leftState) {
            State.CLIMBING -> {
                leftMotor.set(upPower)
            }
            State.LOWERING -> {
                leftMotor.set(downPower)
            }
            State.STEADY_STATE -> {
                leftMotor.set(0.0)
            }
        }

        when(rightState) {
            State.CLIMBING -> {
                rightMotor.set(upPower)
            }
            State.LOWERING -> {
                rightMotor.set(downPower)
            }
            State.STEADY_STATE -> {
                rightMotor.set(0.0)
            }
        }
    }

    internal enum class State {
        CLIMBING, LOWERING, STEADY_STATE
    }
}