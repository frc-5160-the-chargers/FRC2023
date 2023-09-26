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



class RobotContainer {
    private val left1  = neoSparkMax(canBusId = ID.drive_left1){
        inverted = false
    }
    private val left2  = neoSparkMax(canBusId = ID.drive_left2){
        inverted = false
    }
    private val right1 = neoSparkMax(canBusId = ID.drive_right1){
        inverted = true
    }
    private val right2 = neoSparkMax(canBusId = ID.drive_right2){
        inverted = true
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
        neoSparkMax(ID.intake_left) { inverted = false },
        neoSparkMax(ID.intake_right) { inverted = true }
    )
    
    private val driverController = CurvatureDriveController.fromDefaultBindings(
        port = 0,
        driveMultiplier = -0.42,
        rotationMultiplier = -0.3,
        turboModeMultiplierRange = 1.0..2.38,
        precisionModeDividerRange = 1.0..4.0,
        deadband = 0.05
    )
    private val operatorController = OperatorController(port = 1)



    private val proximalMotors = EncoderMotorControllerGroup(
        neoSparkMax(ID.arm_proximal_one){ idleMode = CANSparkMax.IdleMode.kBrake},
        neoSparkMax(ID.arm_proximal_two){idleMode = CANSparkMax.IdleMode.kBrake},
    )

    private val arm = Arm(
        proximalMotors = proximalMotors,
        distalMotor = EncoderMotorControllerGroup(
            // known issue where falcon PID-s back to its original position - fix in progress
            falcon(ID.arm_distal) { neutralMode = NeutralModeValue.Brake }.also{
                // for now, the falocn is manually reconfigured
                val configuration = TalonFXConfiguration()
                configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake
                it.configurator.apply(configuration)
            },

        ),
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


    fun telemetry() {
        SmartDashboard.putNumber("Pitch (º)", navX.gyroscope.pitch.inUnit(degrees))
        SmartDashboard.putNumber("Yaw (º)", navX.gyroscope.yaw.inUnit(degrees))
        SmartDashboard.putNumber("Roll (º)", navX.gyroscope.roll.inUnit(degrees))
    }


    private fun configureSubsystems() {

        arm.q1 += 268.degrees
        arm.q2 += 187.degrees



        println(left1.voltageCompensationNominalVoltage)
        println(left2.voltageCompensationNominalVoltage)
        println(right1.voltageCompensationNominalVoltage)
        println(right2.voltageCompensationNominalVoltage)


        /*
        when no command is scheduled to the arm,
        it calls the moveVoltages function from the output values of the operatorController.
        The values are then logged to SmartDashboard.
         */
        arm.setDefaultRunCommand {
            moveVoltages(operatorController.armVoltages)
            SmartDashboard.putNumber("Joint A desired volts",operatorController.jointAPower * 10.0)
            SmartDashboard.putNumber("Joint B desired volts",operatorController.jointBPower * 10.0)
        }


        /*
        Runs the intake from the operatorController's intakePower getter(returns a value between -1.0 to 1.0)
         */
        intake.setDefaultRunCommand {
            setCustomPower(operatorController.intakePower)
        }

        /*
        The toggle for "milena mode", which allows the operator to control the drivetrain's turning.
         */
        SmartDashboard.putBoolean("milena mode", false)


        /*
        When no command is scheduled to the drivetrain, drives it by default.
         */
        drivetrain.setDefaultRunCommand {
            var turnPower: Double
            // checks if driveStraightAssist is enabled. Toggled via the x button on the driver controller, and requires the navX.
            if (driveStraightAssistEnabled){
                // logs the value, then sets the turnPower to the value obtained from the driveStraightAssistController.
                SmartDashboard.putBoolean("Drive straight assist",true)
                turnPower = driveStraightAssistController.calculateOutput().siValue
            }else{
                SmartDashboard.putBoolean("Drive straight assist",false)


                turnPower = driverController.curvatureOutput.rotationPower
                /*
                Reads the pov output of the operator controller if the driverController's turning output is low enough.
                 */
                if (abs(driverController.curvatureOutput.rotationPower) < 0.1) {
                    SmartDashboard.putBoolean("milena mode", true)
                    turnPower += (-cos((operatorController.povValue + 90.0) * PI / 180.0) * 0.2)
                        .withDeadbandOf(driverController)
                } else {
                    SmartDashboard.putBoolean("milena mode", false)
                }
                /*
                syncs the controller every loop; without it, the controller won't output correct values.
                 */
                driveStraightAssistController.calculateOutput()
            }

            // drives the robot;
            // this code block has the context of the drivetrain, so curvatureDrive is in fact
            // a function belonging to the drivetrain subsystem.
            curvatureDrive(
                driverController.curvatureOutput.xPower,
                turnPower
            )
        }


    }

    private fun configureButtonBindings() {

        driverController.apply{
            // when x is pressed, driveStraightAssist is enabled.
            x{
                onTrue(InstantCommand{
                    driveStraightAssistController.target = navX.heading
                    driveStraightAssistEnabled = true
                })
                onFalse(InstantCommand{driveStraightAssistEnabled = false})
            }
        }

        /*
        Note: Since the arm encoders did not make it on the offseason robot,
        no button bindings for the operator controller are present.
         */

    }



    /*
    The auto chooser for the robot. uses a custom inline function, which is a wrapper around
    SmartDashboard's SendableChooser<Command>.

    the "to" operator is in fact an infix extension function of String, which adds the
    string-command mapping as an option in the dashboard selector.
     */
    private val autoChooser = autoChooser(defaultKey = "drive back") {
        "Taxi and Balance" to drivetrain.taxiBalance(navX)
        "Score, Taxi, Balance" to drivetrain.scoreTaxiBalance(arm, intake, navX)


        with(drivetrain as HeadingProvider) {

            "Score and Taxi" to drivetrain.scoreTaxi(arm, intake)

            // changed: Drives back(intake facing front)
            "drive back" to buildCommand{
                drivetrain.driveStraight(-3.5.meters, -0.2, PIDConstants(0.04, 0.0, 0.0))
            }

        }




    }


    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command?
        get() = autoChooser.selected


}


