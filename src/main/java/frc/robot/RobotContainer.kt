@file:Suppress("detekt.MagicNumber")

package frc.robot

import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.feet
import com.batterystaple.kmeasure.units.inches
import com.batterystaple.kmeasure.units.volts
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.ctre.falcon
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.encoders.ctre.ChargerCANCoder
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.sparkMaxDrivetrain
import frc.robot.commands.HoldArmCartesian
import frc.robot.commands.auto.driveBack
import frc.robot.commands.auto.scoreTaxi
import frc.robot.commands.auto.scoreTaxiBalance
import frc.robot.commands.auto.taxiBalance
import frc.robot.commands.holdAngular
import frc.robot.commands.moveToAngular
import frc.robot.hardware.inputdevices.DriverController
import frc.robot.hardware.inputdevices.OperatorController
import frc.robot.hardware.subsystems.Arm
import frc.robot.hardware.subsystems.Intake
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos


/** The container for the robot. Contains subsystems, OI devices, and commands.  */
class RobotContainer {
    private val left1  = neoSparkMax(canBusId = 12)
    private val left2  = neoSparkMax(canBusId = 11)
    private val right1 = neoSparkMax(canBusId = 15)
    private val right2 = neoSparkMax(canBusId = 9)

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
        voltageCompensationNominalVoltage = 10.volts
    }

    private val intake = Intake(neoSparkMax(8) { inverted = true }, neoSparkMax(22) { inverted = false })

    private val driverController = DriverController(
        port = 0,
        deadband = 0.05,
        forwardsPowerScale = 0.42,
        rotationPowerScale = -0.3,
        turboModeMultiplierRange = 1.0..2.38,
        precisionModeDividerRange = 1.0..4.0,
    )
    private val operatorController = OperatorController(port = 1)

    private val proximalCANCoder = ChargerCANCoder(44)
    private val distalCANCoder = ChargerCANCoder(43)

    private val arm = Arm(
        proximalMotors = EncoderMotorControllerGroup(
            neoSparkMax(7),
        ),
        distalMotor = EncoderMotorControllerGroup(
            falcon(20) { neutralMode = NeutralMode.Brake },
            encoder = distalCANCoder
        ),
        jointAEncoder = proximalCANCoder.absolute,
        jointBEncoder = distalCANCoder.absolute,
        jointAOffset = 0.0.degrees,
        jointBOffset = 0.0.degrees,
        gearRatioA = 1.0 / (8.46 * 70.0/24.0 * 70.0/24.0 * 4.0),
        gearRatioB = 1.0 / (180.0 * 28.0/15.0),
        segmentALength = 37.inches,
        segmentBLength = 19.inches,
        q1SoftRange = 10.degrees..133.degrees,
        q2SoftRange =  0.degrees..0.degrees
    )

    private val navX = NavX()

    init {
        configureSubsystems()
        configureButtonBindings()
    }


    fun telemetry() {
        SmartDashboard.putNumber("Pitch (º)", navX.gyroscope.pitch.inUnit(degrees))
        SmartDashboard.putNumber("Yaw (º)", navX.gyroscope.yaw.inUnit(degrees))
        SmartDashboard.putNumber("Roll (º)", navX.gyroscope.roll.inUnit(degrees))
    }

    var armCommand = arm.holdAngular()

    private fun configureSubsystems() {
        proximalCANCoder.configFactoryDefault()
        proximalCANCoder.setPositionToAbsolute()
        proximalCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
        proximalCANCoder.configMagnetOffset(180.0)

        distalCANCoder.configFactoryDefault()
        distalCANCoder.setPositionToAbsolute()
        distalCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
        distalCANCoder.configMagnetOffset(-50.0)

        arm.q1 += 268.degrees
        arm.q2 += 187.degrees




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

        var cart = false



        arm.setDefaultRunCommand {
            arm.moveVoltages(operatorController.armVoltages)
        }

        operatorController.restPresetButton.whenReleased(InstantCommand { cart = !cart })

        intake.setDefaultRunCommand {
            intake.setCustomPower(operatorController.intakePower)
        }

        SmartDashboard.putBoolean("milena mode", false)


        drivetrain.setDefaultRunCommand {
            var turnPower: Double = driverController.curvatureOutput.rotationPower

            if (abs(driverController.curvatureOutput.rotationPower) < 0.1) {
                SmartDashboard.putBoolean("milena mode", true)
                turnPower += (-cos((operatorController.pov + 90.0) * PI / 180.0) * 0.2)
                                .let { if (abs(it) < 0.05) 0.0 else it }
            } else {
                SmartDashboard.putBoolean("milena mode", false)
            }

            drivetrain.curvatureDrive(
                driverController.curvatureOutput.xPower,
                turnPower
            )
        }

    }

    private fun configureButtonBindings() {
        operatorController.substationConePresetButton.whileHeld(
            arm.moveToAngular(thetaA = 108.degrees, thetaB = -9.degrees))
        operatorController.conePresetButton.whileHeld(
            arm.moveToAngular(thetaA = 58.degrees, thetaB = 32.degrees))
        operatorController.cubePresetButton.whileHeld(
            arm.moveToAngular(thetaA = 60.degrees, thetaB = 9.degrees))
        operatorController.restPresetButton.whileHeld(
            arm.moveToAngular(thetaA = 133.degrees, thetaB = 0.degrees))


        val releaseCommand = InstantCommand {
            println("Setting to thetaA: ${arm.thetaA}, thetaB: ${arm.thetaB}")
            armCommand.thetaA = arm.thetaA
            armCommand.thetaB = arm.thetaB
            println("Set to thetaA: ${armCommand.thetaA}, thetaB: ${armCommand.thetaB}")
        }

        operatorController.substationConePresetButton.whenReleased(releaseCommand)
        operatorController.conePresetButton.whenReleased(releaseCommand)
        operatorController.cubePresetButton.whenReleased(releaseCommand)
        operatorController.restPresetButton.whenReleased(releaseCommand)

    }

    private val autoChooser = SendableChooser<Command>().apply {
        addOption("Taxi and Balance", drivetrain.taxiBalance(navX))
        addOption("Score, Taxi, Balance", drivetrain.scoreTaxiBalance(arm, intake, navX))
        addOption("Score and Taxi",
            with(navX.gyroscope as HeadingProvider) {
                drivetrain.scoreTaxi(arm, intake)
            }
        )
        addOption("TEST hold", HoldArmCartesian(arm))
        addOption("TEST move", HoldArmCartesian(arm, 3.feet, 3.feet))
        setDefaultOption(
            "Drive Back",
            with(navX.gyroscope as HeadingProvider) { drivetrain.driveBack() }
        )
    }.also {
        SmartDashboard.putData("Auto Command", it)
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command? get() = autoChooser.selected



}


