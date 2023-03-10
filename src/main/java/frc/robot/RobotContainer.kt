@file:Suppress("detekt.MagicNumber")

package frc.robot

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.seconds
import com.revrobotics.CANSparkMax
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.RunCommand
import frc.chargers.commands.buildCommand
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.encoders.AnalogPotentiometerPositionEncoder
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.sparkMaxDrivetrain
import frc.robot.commands.HoldArmAngular
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.inputdevices.OperatorController
import frc.robot.hardware.subsystems.Arm
import frc.robot.hardware.subsystems.Intake
import frc.robot.hardware.subsystems.Lights
import kotlin.math.PI
import kotlin.math.cos


/** The container for the robot. Contains subsystems, OI devices, and commands.  */
class RobotContainer {
    val left1  = neoSparkMax(canBusId = 12) // @Tymur change this here
    val left2  = neoSparkMax(canBusId = 11) // and this
    val right1 = neoSparkMax(canBusId = 15)  // and this
    val right2 = neoSparkMax(canBusId = 9)  // and this

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
        voltageCompensationNominalVoltage = 10.0
    }

    val intake = Intake(neoSparkMax(8) { inverted = true }, neoSparkMax(22) { inverted = false })

    private val driverController = DriverController(port = 0, deadband = 0.05, forwardsPowerScale = 0.42, rotationPowerScale = -0.3)
    private val operatorController = OperatorController(port = 1)

    val arm = Arm(
        jointAMotors = EncoderMotorControllerGroup(neoSparkMax(7), neoSparkMax(5)),
        jointBMotor = neoSparkMax(3) { inverted = false },
        jointAEncoder = AnalogPotentiometerPositionEncoder(channel = 1, fullRange = 300.degrees, offset = 0.degrees),
        jointBEncoder = AnalogPotentiometerPositionEncoder(channel = 0, fullRange = 300.degrees, offset = 0.degrees),
        jointAOffset = 0.0.degrees,
        jointBOffset = 0.0.degrees,
        gearRatioA = 1.0 / (8.46 * 70.0/24.0 * 70.0/24.0 * 4.0),
        gearRatioB = 1.0 / (180.0 * 28.0/15.0),
        swivelMotor = neoSparkMax(10),
//        jointAMotor = EncoderMotorControllerGroup(jointAMotor, encoder = encoderASRX),
//        jointBMotor = EncoderMotorControllerGroup(jointBMotor, encoder = encoderBSRX),
//        swivelMotor = brushedSparkMax(),
        segmentALength = 29.inches,
        segmentBLength = 29.5.inches,
    )

    private val lights = Lights()

    var armCommand: Command? = null

    init {
        configureSubsystems()
        configureButtonBindings()
    }

    val camera1 = CameraServer.startAutomaticCapture(0)
    val camera2 = CameraServer.startAutomaticCapture(1)

    private fun configureSubsystems() {
        left1.inverted = true
        left2.inverted = true
        right1.inverted = false
        right2.inverted = false
        println(left1.setIdleMode(CANSparkMax.IdleMode.kBrake))
        println(left2.setIdleMode(CANSparkMax.IdleMode.kBrake))
        println(right1.setIdleMode(CANSparkMax.IdleMode.kBrake))
        println(right2.setIdleMode(CANSparkMax.IdleMode.kBrake))
        println(left1.voltageCompensationNominalVoltage)
        println(left2.voltageCompensationNominalVoltage)
        println(right1.voltageCompensationNominalVoltage)
        println(right2.voltageCompensationNominalVoltage)
        left1.burnFlash()
        left2.burnFlash()
        right1.burnFlash()
        right2.burnFlash()

        arm.setDefaultRunCommand {
            arm.moveVoltages(Arm.JointVoltages(operatorController.leftY, operatorController.rightY))
            arm.rotate((-cos((operatorController.pov + 90.0) * PI/180.0) * 0.05))
        }

        intake.setDefaultRunCommand {
            intake.setCustomPower(operatorController.intakePower)
        }

        driverController.intakeButton.whileHeld({ intake.forward() }, intake)
        driverController.outtakeButton.whileHeld({ intake.reverse() }, intake)
        operatorController.coneButton.whenPressed(InstantCommand { println("setting cone"); lights.setColor(Lights.Color.CONE) })
        operatorController.cubeButton.whenPressed(InstantCommand { println("setting cube"); lights.setColor(Lights.Color.CUBE) })
        operatorController.switchCommandButton.whenPressed(InstantCommand {
            if (armCommand == null) {
                armCommand = HoldArmAngular(arm, thetaA = 230.degrees, thetaB = 382.degrees).apply { schedule() }
            } else {
                armCommand?.cancel()
                armCommand = null
            }
        })

        drivetrain.defaultCommand = RunCommand(drivetrain) {
            drivetrain.curvatureDrive(
                driverController.curvatureOutput.xPower,
                driverController.curvatureOutput.rotationPower
            )
//            drivetrain.tankDrive(driverController.leftY, driverController.rightY)

//            println()
//            println()
//            println("IdleModes:")
//            println(left1.idleMode)
//            println(left2.idleMode)
//            println(right1.idleMode)
//            println(right2.idleMode)
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