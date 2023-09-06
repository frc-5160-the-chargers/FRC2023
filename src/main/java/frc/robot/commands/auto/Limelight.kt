//package frc.robot.commands.auto
//
//import com.batterystaple.kmeasure.quantities.Distance
//import com.batterystaple.kmeasure.quantities.inUnit
//import com.batterystaple.kmeasure.units.degrees
//import com.batterystaple.kmeasure.units.inches
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
//import frc.chargers.commands.buildCommand
//import frc.chargers.controls.pid.PIDConstants
//import frc.chargers.controls.pid.SuperPIDController
//import frc.chargers.hardware.sensors.Limelight
//import frc.chargers.hardware.sensors.NavX
//import frc.chargers.hardware.subsystems.drivetrain.EncoderDifferentialDrivetrain
//import frc.robot.commands.HoldArmCartesian
//import frc.robot.hardware.subsystems.Arm
//
//fun aimingTest(drivetrain: EncoderDifferentialDrivetrain, limelight: Limelight, navX: NavX, targetPipeline: Int) = buildCommand {
//    val aimPID: SuperPIDController = SuperPIDController(
//        pidConstants = PIDConstants(0.02, 0.0, 0.0),
//        getInput = { limelight.tx.inUnit(degrees) },
//        target = 0.0
//    )
//    runParallelUntilOneFinishes {
//        loopForever {
//            SmartDashboard.putNumber("error", aimPID.error)
//            SmartDashboard.putNumber("tx", limelight.tx.inUnit(degrees))
//            SmartDashboard.putNumber("Horizontal Distance", limelight.getHorizontalDistance(targetHeight = 48.inches).inUnit(inches))
//            SmartDashboard.putNumber("Diagonal(actual) Distance", limelight.getDistance(targetHeight = 48.inches).inUnit(inches))
//
//        }
//
//        runSequentially {
//            runOnce {
//                limelight.pipeline = targetPipeline
//            }
//
//            printToConsole("Finished setting pipeline")
//
//            printToConsole("Starting Loop")
//
//            loopUntil({limelight.tv != 0.0},drivetrain){
//                drivetrain.arcadeDrive(0.0,if(navX.gyroscope.yaw < 0.0.degrees){0.2}else{-0.2})
//            }
//
//            printToConsole("finished detection loop. Starting aim loop")
//
//
//            loopUntil({aimPID.error < 0.01}, drivetrain) {
//                drivetrain.arcadeDrive(0.0, -aimPID.calculateOutput())
//            }
//
//
//            printToConsole("Finished aim loop.")
//
//
//        }
//    }
//}
//
//
//fun aimAndScore(arm: Arm, drivetrain: EncoderDifferentialDrivetrain, limelight: Limelight, navX: NavX, pipeline: Int, targetHeight: Distance, verticalOffset: Distance, horizontalOffset: Distance) = buildCommand{
//
//    +aimingTest(drivetrain, limelight, navX, 0)
//
//    // note: getHorizontalDistance is currently returning a double; this is a bug and it should be fixed
//    val horizontalDistance by getOnceDuringRun{limelight.getHorizontalDistance(targetHeight)}
//
//    +HoldArmCartesian(arm,horizontalDistance - horizontalOffset,targetHeight-verticalOffset, forwardTolerance = 1.inches, upTolerance = 1.inches)
//
//}