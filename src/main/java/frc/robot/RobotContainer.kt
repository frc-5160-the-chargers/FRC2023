@file:Suppress("detekt.MagicNumber")

package frc.robot

import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.seconds
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.RunCommand
import frc.chargers.commands.buildCommand
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.sparkMaxDrivetrain
import frc.robot.hardware.inputdevices.DriverController

/** The container for the robot. Contains subsystems, OI devices, and commands.  */
class RobotContainer {
    val left1 = neoSparkMax(canBusId = 7) // @Tymur change this here
    val left2 = neoSparkMax(canBusId = 11) // and this
    val right1 = neoSparkMax(canBusId = 9) // and this
    val right2 = neoSparkMax(canBusId = 3) // and this

    //    // The robot's subsystems and commands are defined here...
    private val drivetrain: EncoderDifferentialDrivetrain = sparkMaxDrivetrain(
        leftMotors = EncoderMotorControllerGroup(
            left1,
            left2
        ),
        rightMotors = EncoderMotorControllerGroup(
            right1,
            right2,
        ),
        invertMotors = false,
        gearRatio = 1.0/10.71,
        wheelDiameter = 6.inches,
        width = 27.inches
    ) {
        idleMode = CANSparkMax.IdleMode.kBrake
    }

    private val driverController = DriverController(port = 0, deadband = 0.05, forwardsPowerScale = 0.35, rotationPowerScale = -0.4)
//    private val operatorController = OperatorController(port = 1)

    init {
        configureSubsystems()
        configureButtonBindings()
    }

    private fun configureSubsystems() {
        left1.inverted = true
        left2.inverted = true
        right1.inverted = false
        right2.inverted = false
        println(left1.setIdleMode(CANSparkMax.IdleMode.kBrake))
        println(left2.setIdleMode(CANSparkMax.IdleMode.kBrake))
        println(right1.setIdleMode(CANSparkMax.IdleMode.kBrake))
        println(right2.setIdleMode(CANSparkMax.IdleMode.kBrake))
        left1.burnFlash()
        left2.burnFlash()
        right1.burnFlash()
        right2.burnFlash()



        drivetrain.defaultCommand = RunCommand(drivetrain) {
            drivetrain.curvatureDrive(
                driverController.curvatureOutput.xPower,
                driverController.curvatureOutput.rotationPower
            )
//            drivetrain.tankDrive(driverController.leftY, driverController.rightY)

            println()
            println()
            println("IdleModes:")
            println(left1.idleMode)
            println(left2.idleMode)
            println(right1.idleMode)
            println(right2.idleMode)
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {}

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