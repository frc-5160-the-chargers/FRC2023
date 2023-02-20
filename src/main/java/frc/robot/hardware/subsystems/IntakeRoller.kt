package frc.robot.hardware.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase

class IntakeRoller(private val intakeMotor: CANSparkMax) : SubsystemBase() {
    private var state: State? = null
    private var power = 0.0

    init {
        reset()
    }

    fun reset() {
        state = State.STOPPED
    }

    fun intake() {
        state = State.INTAKING
    }

    fun outtake() {
        state = State.OUTTAKING
    }

    fun stop() {
        state = State.STOPPED
    }

    fun setMotorRaw(power: Double) {
        this.power = power
    }

    override fun periodic() {
        // switch (state){
        //     case INTAKING:
        //         intakeMotor.set(intakeRollerConstants.rollerPower);
        //     case OUTTAKING:
        //         intakeMotor.set(-intakeRollerConstants.rollerPower);
        //     case STOPPED:
        //         intakeMotor.stopMotor();
        // }
        intakeMotor.set(power)
    }

    internal enum class State(val value: Int) {
        STOPPED(0), INTAKING(1), OUTTAKING(2);
    }
}