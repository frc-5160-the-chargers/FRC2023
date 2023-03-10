package frc.robot.hardware.subsystems

import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Lights : SubsystemBase() {
    private val coneOutput = DigitalOutput(0)
    private val cubeOutput = DigitalOutput(1)

    fun setColor(color: Color) {
        when (color) {
            Color.CONE -> {
                coneOutput.set(true)
                cubeOutput.set(false)
            }
            Color.CUBE -> {
                cubeOutput.set(true)
                coneOutput.set(false)
            }
        }
    }

    enum class Color {
        CONE, CUBE
    }
}