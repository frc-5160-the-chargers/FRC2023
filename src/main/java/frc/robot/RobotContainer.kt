@file:Suppress("detekt.MagicNumber")

package frc.robot


import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj2.command.Command
import frc.chargers.commands.DoNothing
import frc.chargers.commands.setDefaultRunCommand
import frc.chargers.hardware.inputdevices.CurvatureDriveController
import frc.chargers.hardware.motorcontrol.EncoderMotorControllerGroup
import frc.chargers.hardware.subsystems.drivetrain.*
import frc.robot.hardware.inputdevices.OperatorController
import frc.robot.hardware.subsystems.arm.Arm
import frc.robot.hardware.subsystems.arm.ArmIOReal
import frc.robot.hardware.subsystems.arm.ArmIOSim
//import frc.robot.hardware.subsystems.intake.Intake
//import frc.robot.hardware.subsystems.intake.IntakeIOReal
//import frc.robot.hardware.subsystems.intake.IntakeIOSim



object RobotContainer {

    private val drivetrain: EncoderDifferentialDrivetrain
    private val arm: Arm
    //private val intake: Intake

    init{
        if (RobotBase.isReal()){
            drivetrain = sparkMaxDrivetrain(
                leftMotors = EncoderMotorControllerGroup(
                    DriveMotors.left1, DriveMotors.left2
                ),
                rightMotors = EncoderMotorControllerGroup(
                    DriveMotors.right1, DriveMotors.right2
                ),
                gearRatio = DrivetrainConstants.gearRatio,
                wheelDiameter = DrivetrainConstants.wheelDiameter,
                width = DrivetrainConstants.width
            )
            arm = Arm(ArmIOReal)
            //intake = Intake(IntakeIOReal)
        }else{
            drivetrain = simulatedDrivetrain(
                simMotors = DifferentialDrivetrainSim.KitbotMotor.kDoubleNEOPerSide,
                wheelDiameter = DrivetrainConstants.wheelDiameter,
                width = DrivetrainConstants.width
            )
            arm = Arm(ArmIOSim)
            //intake = Intake(IntakeIOSim)
        }
        configureButtonBindings()
    }

    private fun configureButtonBindings(){
        drivetrain.setDefaultRunCommand{
            curvatureDrive(driverController.curvatureOutput)
        }
        arm.setDefaultRunCommand{
            moveVoltages(operatorController.armVoltages)
        }
        /*
        intake.setDefaultRunCommand{
            setSpeed(operatorController.intakePower)
        }

         */
    }

    fun telemetry(){

    }

    
    private val driverController = CurvatureDriveController.fromDefaultBindings(
        port = 0,
        driveMultiplier = -0.42,
        rotationMultiplier = -0.3,
        turboModeMultiplierRange = 1.0..2.38,
        precisionModeDividerRange = 1.0..4.0,
        deadband = 0.05
    )
    private val operatorController: OperatorController = OperatorController(1)



    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() = DoNothing()


}


