@file:Suppress("detekt.MagicNumber")

package frc.robot

import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.*
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.chargers.commands.InstantCommand
import frc.chargers.commands.buildCommand
import frc.chargers.commands.drivetrainCommands.driveStraight
//import frc.chargers.commands.drivetrainCommands.followPath
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.controls.pid.PIDConstants
import frc.chargers.controls.pid.UnitSuperPIDController
import frc.chargers.hardware.inputdevices.CurvatureDriveController
import frc.chargers.hardware.inputdevices.withDeadbandOf
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.motorcontrol.ctre.falcon
import frc.chargers.hardware.motorcontrol.rev.neoSparkMax
import frc.chargers.hardware.sensors.NavX
import frc.chargers.hardware.sensors.encoders.absolute.ChargerCANcoder
import frc.chargers.hardware.sensors.gyroscopes.HeadingProvider
import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
import frc.chargers.hardware.subsystems.drivetrain.sparkMaxDrivetrain
import frc.chargers.wpilibextensions.autoChooser
import frc.chargers.wpilibextensions.dashboardChooser
import frc.chargers.wpilibextensions.geometry.LinearTrapezoidProfile
//import frc.robot.commands.HoldArmCartesian
import frc.robot.commands.auto.scoreTaxi
import frc.robot.commands.auto.scoreTaxiBalance
import frc.robot.commands.auto.taxiBalance
//import frc.robot.commands.moveToAngular
import frc.robot.hardware.inputdevices.OperatorController
import frc.robot.hardware.subsystems.Arm
import frc.robot.hardware.subsystems.Intake
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos


/**
 * Test
 */
class RobotContainer {
    private val left1  = neoSparkMax(canBusId = ID.drive_left1){
        inverted = true
    }
    private val left2  = neoSparkMax(canBusId = ID.drive_left2){
        inverted = true
    }
    private val right1 = neoSparkMax(canBusId = ID.drive_right1){
        inverted = false
    }
    private val right2 = neoSparkMax(canBusId = ID.drive_right2){
        inverted = false
    }

    private val leftMotors = EncoderMotorControllerGroup(
        left1,
        left2
    )

    private val rightMotors = EncoderMotorControllerGroup(
        right1,
        right2,
    )


    //    // The robot's subsystems and commands are defined here...
    private val drivetrain: EncoderDifferentialDrivetrain = sparkMaxDrivetrain(
        leftMotors = leftMotors,
        rightMotors = rightMotors,
        invertMotors = false,
        gearRatio = 1.0/10.71,
        wheelDiameter = 6.inches,
        width = 27.inches
    ) {
        idleMode = CANSparkMax.IdleMode.kBrake
        voltageCompensationNominalVoltage = 10.volts
    }

    private val intake = Intake(
        neoSparkMax(ID.intake_left) { inverted = true },
        neoSparkMax(ID.intake_right) { inverted = false },
        passiveSpeed = -0.1
    )

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

    /*
    private val proximalCANCoder = ChargerCANcoder(ID.arm_jointa_encoder_proximal){
        absoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1
        magnetOffset = 180.degrees
    }
    private val distalCANCoder = ChargerCANcoder(ID.arm_jointb_encoder_distal){
        absoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1
        magnetOffset = -(50.degrees)
    }
     */

    private val proximalMotors = EncoderMotorControllerGroup(
        neoSparkMax(ID.arm_proximal_one),
        neoSparkMax(ID.arm_proximal_two){inverted = true; idleMode = CANSparkMax.IdleMode.kBrake},
    )

    private val arm = Arm(
        proximalMotors = proximalMotors,
        distalMotor = EncoderMotorControllerGroup(
            falcon(ID.arm_distal) { neutralMode = NeutralModeValue.Brake }.also{
                val configuration = TalonFXConfiguration()
                configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake
                it.configurator.apply(configuration)
            },
            /*encoder = distalCANCoder*/
        ),
        /*
        jointAEncoder = proximalCANCoder.absolute,
        jointBEncoder = distalCANCoder.absolute,
         */
        jointAOffset = 0.0.degrees,
        jointBOffset = 0.0.degrees,
        gearRatioA = 1.0 / (8.46 * 70.0/24.0 * 70.0/24.0 * 4.0),
        // 125 is the falcon gear ratio(was 180 went to 135, and is now 125)
        gearRatioB = 1.0 / (125.0 * 28.0/15.0),
        segmentALength = 37.inches,
        segmentBLength = 19.inches,
        q1SoftRange = 10.degrees..133.degrees,
        q2SoftRange =  0.degrees..0.degrees
    )

    private val navX = NavX()
//    private val limelight = Limelight()

    private val driveStraightAssistController =
        UnitSuperPIDController(
            PIDConstants(0.2,0.0,0.0),
            {navX.heading},
            outputRange = Scalar(-1.0)..Scalar(1.0),
            target = navX.heading
        )
    private var driveStraightAssistEnabled: Boolean = false



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



        println(left1.voltageCompensationNominalVoltage)
        println(left2.voltageCompensationNominalVoltage)
        println(right1.voltageCompensationNominalVoltage)
        println(right2.voltageCompensationNominalVoltage)
        left1.burnFlash()
        left2.burnFlash()
        right1.burnFlash()
        right2.burnFlash()


        arm.setDefaultRunCommand {
            // checks if all abort is enabled!
            if (allAbortChooser.selected){
                moveSpeeds(0.0,0.0)
            }else{
                moveVoltages(operatorController.armVoltages)
            }
            SmartDashboard.putNumber("Joint A desired volts",operatorController.jointAPower * 10.0)
            SmartDashboard.putNumber("Joint B desired volts",operatorController.jointBPower * 10.0)
        }


        intake.setDefaultRunCommand {
            if (allAbortChooser.selected){
                setCustomPower(0.0)
            }else{
                setCustomPower(operatorController.intakePower)
            }
        }

        SmartDashboard.putBoolean("milena mode", false)

//        val driverStraightAssistTurnPID = UnitSuperPIDController<AngleDimension, ScalarDimension>(
//            pidConstants = PIDConstants(0.5, 0.0, 0.0),
//            getInput = { navX.heading },
//            target = navX.heading
//        )

        drivetrain.setDefaultRunCommand {
            var turnPower: Double
            if (driveStraightAssistEnabled){
                SmartDashboard.putBoolean("Drive straight assist",true)
                turnPower = driveStraightAssistController.calculateOutput().siValue
            }else{
                SmartDashboard.putBoolean("Drive straight assist",false)

                turnPower = driverController.curvatureOutput.rotationPower
                if (abs(driverController.curvatureOutput.rotationPower) < 0.1) {
                    SmartDashboard.putBoolean("milena mode", true)
                    turnPower += (-cos((operatorController.povValue + 90.0) * PI / 180.0) * 0.2)
                        .withDeadbandOf(driverController)
                } else {
                    SmartDashboard.putBoolean("milena mode", false)
                }
                // syncs the controller every loop
                driveStraightAssistController.calculateOutput()
            }

            curvatureDrive(
                driverController.curvatureOutput.xPower,
                turnPower
            )
        }


    }

    private fun configureButtonBindings() {

        driverController.apply{
            x{
                onTrue(InstantCommand{
                    driveStraightAssistController.target = navX.heading
                    driveStraightAssistEnabled = true
                })
                onFalse(InstantCommand{driveStraightAssistEnabled = false})
            }
        }
        /*
    val conePresetButton = button(Button.kX)
    val cubePresetButton = button(Button.kY)
    val substationConePresetButton = button(Button.kB)
    val restPresetButton = button(Button.kA)

    val shiftButton = button(Button.kLeftBumper)

     */

        operatorController.apply{
            a{
                onTrue(InstantCommand{
                    intake.passiveSpeedEnabled = !intake.passiveSpeedEnabled
                })
            }
        }


        /*
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
                whileTrue(arm.moveToAngular(thetaA = 133.degrees, thetaB = 0.degrees))
            }

            // back intake preset: tbd
            backButton{
                whileTrue(arm.moveToAngular(thetaA = 0.degrees, thetaB = 0.degrees))
            }
        }
         */

    }

    private val allAbortChooser = dashboardChooser(name = "ALL ABORT for robot", "DISABLED" to false){
        "ENABLE ABORT!" to true
    }


    private val autoChooser = autoChooser(defaultKey = "drive back") {
        "Taxi and Balance" to drivetrain.taxiBalance(navX)
        "Score, Taxi, Balance" to drivetrain.scoreTaxiBalance(arm, intake, navX)


        with(drivetrain as HeadingProvider) {

            "Score and Taxi" to drivetrain.scoreTaxi(arm, intake)

            "drive back" to buildCommand{
                drivetrain.driveStraight(3.5.meters, 0.2, PIDConstants(0.04, 0.0, 0.0))
            }

        }




    }

    /*
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
            buildCommand{
                with(navX.gyroscope as HeadingProvider) {
                    drivetrain.driveStraight(3.5.meters, 0.2, PIDConstants(0.04, 0.0, 0.0))
                }
            }
        )
         */

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */

    /*
    val autonomousCommand: Command? get() = autoChooser.selected.also{
        SmartDashboard.putBoolean("auto is not null!", it != null)
    }
     */


    val autonomousCommand: Command?
        get() = buildCommand{
            loopForever(drivetrain){
                leftMotors.set(0.3)
            }
        }






}


