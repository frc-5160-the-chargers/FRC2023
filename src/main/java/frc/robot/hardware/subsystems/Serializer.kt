package frc.robot.hardware.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Serializer(private val motor: CANSparkMax) : SubsystemBase() {
    private var power: Double
    private var state: State
    var auto: Boolean

    init {
        state = State.DISABLED
        power = 0.0
        auto = false
    }

    //This exists so that all subsystems have a commonly-named reset method
    fun reset() {
        disable()
    }

    fun runForward() {
        state = State.FORWARD
    }

    fun runReverse() {
        state = State.REVERSE
    }

    fun disable() {
        state = State.DISABLED
    }

    val isEnabled: Boolean
        get() = state == State.FORWARD

    fun setPowerRaw(power: Double) {
        this.power = power
    }

    override fun periodic() {
        // switch (state){
        //     case FORWARD:
        //         power = serializerConstants.enablePower;
        //     case REVERSE:
        //         power = -serializerConstants.enablePower;
        //     case DISABLED:
        //         power = 0;
        // }
        motor.set(power)
        //        System.out.println("Running serializer at " + power);
    }

    internal enum class State(val value: Int) {
        FORWARD(0), REVERSE(1), DISABLED(2);
    }
}