@file:Suppress("detekt.MagicNumber")

package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.buildCommand
import frc.chargers.commands.drivetrainCommands.followPath
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.hardware.inputdevices.CurvatureDriveController
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.ctre.falcon
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.sparkMaxDrivetrain
import frc.chargers.wpilibextensions.autoChooser
import frc.chargers.wpilibextensions.geometry.LinearTrapezoidProfile
import frc.robot.commands.HoldArmCartesian
import frc.robot.commands.auto.driveBack
import frc.robot.commands.auto.scoreTaxi
import frc.robot.commands.auto.scoreTaxiBalance
import frc.robot.commands.auto.taxiBalance
import frc.robot.commands.moveToAngular
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

    /*
    private val driverController = DriverController(
        port = 0,
        deadband = 0.05,
        forwardsPowerScale = 0.42,
        rotationPowerScale = -0.3,
        turboModeMultiplierRange = 1.0..2.38,
        precisionModeDividerRange = 1.0..4.0,
    )
     */
    
    private val driverController = CurvatureDriveController.fromDefaultBindings(
        port = 0,
        driveMultiplier = 0.42,
        rotationMultiplier = -0.3,
        turboModeMultiplierRange = 1.0..2.38,
        precisionModeDividerRange = 1.0..4.0,
        deadband = 0.05
    )
    private val operatorController = OperatorController(port = 1)

    private val proximalCANCoder = ChargerCANcoder(44){
        absoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1
        magnetOffset = 180.degrees
    }
    private val distalCANCoder = ChargerCANcoder(43){
        absoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1
        magnetOffset = -(50.degrees)
    }

    private val arm = Arm(
        proximalMotors = EncoderMotorControllerGroup(
            neoSparkMax(7),
//            neoSparkMax(4),
        ),
        distalMotor = EncoderMotorControllerGroup(
            falcon(20) { neutralMode = NeutralModeValue.Brake },
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
//    private val limelight = Limelight()

    init {
        configureSubsystems()
        configureButtonBindings()
    }

//    val camera = CameraServer.startAutomaticCapture()
//    val camera1 = CameraServer.startAutomaticCapture(0)
//    val camera2 = CameraServer.startAutomaticCapture(1)

    fun telemetry() {
        SmartDashboard.putNumber("Pitch (º)", navX.gyroscope.pitch.inUnit(degrees))
        SmartDashboard.putNumber("Yaw (º)", navX.gyroscope.yaw.inUnit(degrees))
        SmartDashboard.putNumber("Roll (º)", navX.gyroscope.roll.inUnit(degrees))
    }


    private fun configureSubsystems() {


//        arm.q1 += 268.85.degrees
//        arm.q2 += 177.02.degrees

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


        arm.setDefaultRunCommand {
            moveVoltages(operatorController.armVoltages)
        }


        intake.setDefaultRunCommand {
            setCustomPower(operatorController.intakePower)
        }

        SmartDashboard.putBoolean("milena mode", false)

//        val driverStraightAssistTurnPID = UnitSuperPIDController<AngleDimension, ScalarDimension>(
//            pidConstants = PIDConstants(0.5, 0.0, 0.0),
//            getInput = { navX.heading },
//            target = navX.heading
//        )

        drivetrain.setDefaultRunCommand {
            var turnPower: Double = driverController.curvatureOutput.rotationPower

            if (abs(driverController.curvatureOutput.rotationPower) < 0.1) {
                SmartDashboard.putBoolean("milena mode", true)
                turnPower += (-cos((operatorController.povValue + 90.0) * PI / 180.0) * 0.2)
                                .let { if (abs(it) < 0.05) 0.0 else it }
            } else {
                SmartDashboard.putBoolean("milena mode", false)
            }

            curvatureDrive(
                driverController.curvatureOutput.xPower,
                turnPower
            )
        }


    }

    private fun configureButtonBindings() {
        /*
    val conePresetButton = button(Button.kX)
    val cubePresetButton = button(Button.kY)
    val substationConePresetButton = button(Button.kB)
    val restPresetButton = button(Button.kA)

    val shiftButton = button(Button.kLeftBumper)

     */


        operatorController.apply{
            // cone preset button
            x{
                whileTrue(arm.moveToAngular(thetaA = 58.degrees, thetaB = 32.degrees))
            }
            // cube preset button
            y{
                whileTrue(arm.moveToAngular(thetaA = 60.degrees, thetaB = 9.degrees))
            }
            // substation cone preset button
            b{
                whileTrue(arm.moveToAngular(thetaA = 108.degrees, thetaB = -9.degrees))
            }
            // rest preset button
            a{
                onTrue(arm.moveToAngular(thetaA = 133.degrees, thetaB = 0.degrees))
            }
        }

    }


    private val autoChooser = autoChooser {
        addOption("Taxi and Balance", drivetrain.taxiBalance(navX))
        addOption("Score, Taxi, Balance", drivetrain.scoreTaxiBalance(arm, intake, navX))
        addOption("Score and Taxi",
            with(navX.gyroscope as HeadingProvider) {
                drivetrain.scoreTaxi(arm, intake)
            }
        )
//        addOption("Limelight", aimAndScore(arm, drivetrain, limelight, navX,0, 48.inches, 0.meters, 1.7.centi.meters))
        addOption("TEST hold", HoldArmCartesian(arm))
        addOption("TEST move", HoldArmCartesian(arm, 3.feet, 3.feet))
        addOption("TEST follow path", buildCommand {
            drivetrain.followPath(
                "pathplanner_test",
                LinearTrapezoidProfile.Constraints(1.0.ofUnit(meters/seconds), 1.0.ofUnit(meters/seconds/seconds)),
                true
            )
        })
        setDefaultOption(
            "Drive Back",
            with(navX.gyroscope as HeadingProvider) { drivetrain.driveBack() }
        )
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command? get() = autoChooser.selected



}


