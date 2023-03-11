@file:Suppress("detekt.MagicNumber")

package frc.robot

import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.revrobotics.CANSparkMax
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.RunCommand
import frc.chargers.commands.buildCommand
import frc.chargers.commands.drivetrainCommands.driveStraight
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.encoders.AnalogPotentiometerPositionEncoder
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.sparkMaxDrivetrain
import frc.robot.commands.HoldArmAngular
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.inputdevices.OperatorController
import frc.robot.hardware.subsystems.Arm
import frc.robot.hardware.subsystems.Intake
import frc.robot.hardware.subsystems.Lights
import frc.robot.math.sin
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
//    private val navX = NavX()

    var armCommand: Command? = null

    init {
        configureSubsystems()
        configureButtonBindings()
    }

    val camera = CameraServer.startAutomaticCapture()
//    val camera1 = CameraServer.startAutomaticCapture(0)
//    val camera2 = CameraServer.startAutomaticCapture(1)

    val conePresetCommand = HoldArmAngular(arm, thetaA = 216.degrees, thetaB = 355.degrees)
    val cubePresetCommand = HoldArmAngular(arm, thetaA = 206.degrees, thetaB = 302.degrees)
    val substationPresetCommand = HoldArmAngular(arm, thetaA = 142.degrees, thetaB = 167.degrees)
    val restPresetCommand = HoldArmAngular(arm, thetaA = 109.degrees, thetaB = 122.degrees)

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
            arm.moveVoltages(Arm.JointVoltages(operatorController.leftY * 6.0, operatorController.rightY * 6.0))
            arm.rotate((-cos((operatorController.pov + 90.0) * PI/180.0) * 0.05))
        }

        intake.setDefaultRunCommand {
            intake.setCustomPower(operatorController.intakePower)
        }

        driverController.intakeButton.whileHeld({ intake.intake() }, intake)
        driverController.outtakeButton.whileHeld({ intake.outtake() }, intake)
        operatorController.coneLightButton.whenPressed(InstantCommand { println("setting cone"); lights.setColor(Lights.Color.CONE) })
        operatorController.cubeLightButton.whenPressed(InstantCommand { println("setting cube"); lights.setColor(Lights.Color.CUBE) })

//        operatorController.conePresetButton.whenPressed(InstantCommand {
//            if (armCommand == null) {
//                armCommand = HoldArmAngular(arm, thetaA = 242.degrees, thetaB = 242.degrees + 146.degrees).apply { schedule() }
//            } else {
//                armCommand?.cancel()
//                armCommand = null
//            }
//        })
//
//        operatorController.cubePresetButton.whenPressed(InstantCommand {
//            if (armCommand == null) {
//                armCommand = HoldArmAngular(arm, thetaA = 182.degrees, thetaB = 182.degrees + 32.degrees).apply { schedule() }
//            } else {
//                armCommand?.cancel()
//                armCommand = null
//            }
//        })

        // QQQQQQQQQQQQ
        // Stow: 105, 6
        // Cube: 182, 58
        // Substation_mid: 159, 32
        // Substation_close: 127, 23

        // θθθθθθθθθθθθ
        // Cone: 216, 355
        // Cube: 206, 302
        // Stow: 109, 122
        // Substation_mid: 142, 167
        // Substation_close: na

        // Soft stop: Q1 = 91

        operatorController.conePresetButton.whileHeld(conePresetCommand)
        operatorController.cubePresetButton.whileHeld(cubePresetCommand)
        operatorController.substationPresetButton.whileHeld(substationPresetCommand)
        operatorController.restPresetButton.whileHeld(restPresetCommand)


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
//    val autonomousCommand: Command get() = buildCommand {runForever(drivetrain) { drivetrain.curvatureDrive(0.0, 0.0)} }
    val autonomousCommand: Command get() = buildCommand {
        drivetrain.driveStraight(-6.inches, -0.2, PIDConstants(0.04, 0.0, 0.0))
        runFor(5.seconds, HoldArmAngular(arm, thetaB = 302.degrees))
        runFor(5.seconds, HoldArmAngular(arm, thetaA = 206.degrees))
        drivetrain.driveStraight(6.inches, 0.2, PIDConstants(0.04, 0.0, 0.0))
        waitFor(1.seconds)
        runFor(3.seconds, RunCommand(intake) { intake.outtake() })
        drivetrain.driveStraight(-2.meters, 0.2, PIDConstants(0.04, 0.0, 0.0))
    }

//        balanceRobotCommand
//        get() = HoldArmAngular(arm,206.degrees,302.degrees)
//            .andThen(InstantCommand(intake) { intake.intake() })
//            .andThen(WaitCommand(0.25))
//            .andThen(InstantCommand(intake) { intake.disable() })
//            .andThen(driveReverseUntilInclineHitCommand)
//            .andThen(balanceRobotCommand)

    val driveReverseUntilInclineHitCommand = buildCommand {
        runUntil({ navX.gyroscope.pitch > 10.0.degrees || navX.gyroscope.pitch < 10.0.degrees }, drivetrain) {
            drivetrain.curvatureDrive(-0.5,0.0)
        }
    }



    val navX = NavX()
    private val balanceRobotCommand = RunCommand(drivetrain) {
        drivetrain.arcadeDrive(-sin(navX.gyroscope.pitch),0.0)
    }

}