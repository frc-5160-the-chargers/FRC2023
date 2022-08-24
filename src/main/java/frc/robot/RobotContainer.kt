@file:Suppress("detekt.MagicNumber")

package frc.robot

import com.batterystaple.kmeasure.units.inches
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.chargers.commands.DoNothing
import frc.chargers.commands.RunCommand
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.ctre.falcon
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.curvatureDrive
import frc.chargers.hardware.subsystems.drivetrain.sparkMaxDrivetrain
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.inputdevices.OperatorController
import frc.robot.hardware.subsystems.Shooter

class RobotContainer {
    // The robot's subsystems and commands are defined here...
    //    private final NavX navx = new NavX();
    private val drivetrain: EncoderDifferentialDrivetrain = sparkMaxDrivetrain(
        leftMotors = EncoderMotorControllerGroup(
            neoSparkMax(canBusId = 9),
            neoSparkMax(canBusId = 5)
        ),
        rightMotors = EncoderMotorControllerGroup(
            neoSparkMax(canBusId = 1),
            neoSparkMax(canBusId = 3)
        ),
        invertMotors = true,
        gearRatio = 1.0/10.71,
        wheelDiameter = 6.inches,
        width = 27.inches
    ) {
        idleMode = CANSparkMax.IdleMode.kBrake
    }

    private val shooter: Shooter = Shooter(
        frontMotor = falcon(canId = 6),
        backMotor = falcon(canId = 7),
        frontPower = 0.5,
        backPower = 1.0
    )

    val driverController = DriverController(port = 0, 0.05)
    val operatorController = OperatorController(port = 1)

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // Configure the button bindings
        configureButtonBindings()

        drivetrain.defaultCommand = RunCommand(drivetrain) {
            drivetrain.curvatureDrive(driverController.curvatureOutput)
        }

        shooter.defaultCommand = RunCommand(shooter) {
            shooter.disable()
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {
        operatorController.toggleShooter
            .whenPressed(InstantCommand(shooter::toggle, shooter))
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() = DoNothing()
}