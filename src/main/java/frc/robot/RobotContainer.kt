@file:Suppress("detekt.MagicNumber")

package frc.robot

import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.seconds
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.commands.RunCommand
import frc.chargers.commands.buildCommand
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.ctre.falcon
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.sparkMaxDrivetrain
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.inputdevices.OperatorController
import frc.robot.hardware.subsystems.Climber
import frc.robot.hardware.subsystems.IntakeRoller
import frc.robot.hardware.subsystems.Serializer
import frc.robot.hardware.subsystems.Shooter

/** The container for the robot. Contains subsystems, OI devices, and commands.  */
class RobotContainer {
    val l1 = neoSparkMax(canBusId = 7)

    val l2 = neoSparkMax(canBusId = 11)

    val r1 = neoSparkMax(canBusId = 9)

    val r2 = neoSparkMax(canBusId = 3)

    //    // The robot's subsystems and commands are defined here...
    private val drivetrain: EncoderDifferentialDrivetrain = sparkMaxDrivetrain(
        leftMotors = EncoderMotorControllerGroup(
            l1,
            l2
        ),
        rightMotors = EncoderMotorControllerGroup(
            r1,
            r2,
        ),
        invertMotors = false,
        gearRatio = 1.0/10.71,
        wheelDiameter = 6.inches,
        width = 27.inches
    ) {
        idleMode = CANSparkMax.IdleMode.kBrake
    }

    private val shooter: Shooter = Shooter(
        frontMotor = falcon(canId = 6),
        backMotor = falcon(canId = 7),
        frontPower = -0.45,
        backPower = -0.9
    )

    val serializerMotor = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushed)
    private val serializer: Serializer = Serializer(serializerMotor)

    val intakeMotor = CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushed)
    private val intakeRoller: IntakeRoller = IntakeRoller(intakeMotor)

    private val climber: Climber = Climber(
        leftMotor = neoSparkMax(canBusId = 5),
        rightMotor = neoSparkMax(canBusId = 8) { inverted = true },
        upPower = 0.2,
        downPower = -0.2,
    )

    private val driverController = DriverController(port = 0, deadband = 0.05)
    private val operatorController = OperatorController(port = 1)

    init {
        configureSubsystems()
        configureButtonBindings()
    }

    private fun configureSubsystems() {
        l1.inverted = true
        l2.inverted = true
        r1.inverted = false
        r2.inverted = false
        println(l1.setIdleMode(CANSparkMax.IdleMode.kBrake))
        println(l2.setIdleMode(CANSparkMax.IdleMode.kBrake))
        println(r1.setIdleMode(CANSparkMax.IdleMode.kBrake))
        println(r2.setIdleMode(CANSparkMax.IdleMode.kBrake))
        l1.burnFlash()
        l2.burnFlash()
        r1.burnFlash()
        r2.burnFlash()



        drivetrain.defaultCommand = RunCommand(drivetrain) {
            drivetrain.curvatureDrive(driverController.curvatureOutput.xPower, driverController.curvatureOutput.rotationPower)
//            drivetrain.tankDrive(driverController.leftY, driverController.rightY)

            println()
            println()
            println("IdleModes:")
            println(l1.idleMode)
            println(l2.idleMode)
            println(r1.idleMode)
            println(r2.idleMode)
        }

        climber.defaultCommand = RunCommand(climber) {
            climber.stop()
        }

        shooter.defaultCommand = RunCommand(shooter) {
            shooter.disable()
        }

        serializer.defaultCommand = RunCommand(serializer) {
            serializer.reset()
            serializer.setPowerRaw(0.0)
        }

        intakeRoller.defaultCommand = RunCommand(intakeRoller) {
            intakeRoller.reset()
            intakeRoller.setMotorRaw(0.0)
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {
        operatorController.enableShooterButton
            .whileHeld(InstantCommand(shooter::enable, shooter))

        driverController.raiseLeftArmButton
            .whileHeld(InstantCommand(climber::climbLeft, climber))

        driverController.raiseRightArmButton
            .whileHeld(InstantCommand(climber::climbRight, climber))

        driverController.lowerLeftArmButton
            .whileHeld(InstantCommand(climber::lowerLeft, climber))

        driverController.lowerRightArmButton
            .whileHeld(InstantCommand(climber::lowerRight, climber))

        operatorController.intakeButton
            .whileHeld(InstantCommand( { intakeRoller.setMotorRaw(-0.6) }, intakeRoller))

        operatorController.outtakeButton
            .whileHeld(InstantCommand({ intakeRoller.setMotorRaw(0.6) }, intakeRoller))

        operatorController.runSerializerForwardButton
            .whileHeld(InstantCommand({ serializer.auto = false; serializer.setPowerRaw(0.5) }, serializer))

        operatorController.runSerializerForwardButton
            .whileHeld(InstantCommand({ serializer.auto = true; serializer.setPowerRaw(-0.5) }, serializer))
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() = buildCommand {
            runSequentially {
                runFor(4.5.seconds, drivetrain) {
                    drivetrain.arcadeDrive(power = 0.25, rotation = 0.0)
                }

                runOnce(drivetrain) {
                    drivetrain.arcadeDrive(0.0, 0.0)
                }
            }
        }
}