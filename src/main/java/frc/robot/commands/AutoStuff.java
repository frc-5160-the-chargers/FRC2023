// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.chargers.hardware.subsystems.drivetrain.BasicDifferentialDrivetrain;
import frc.robot.hardware.subsystems.Serializer;
import frc.robot.hardware.subsystems.Shooter;

public class AutoStuff extends CommandBase {
  private final Shooter shooter;
  private final Serializer serializer;
  private final BasicDifferentialDrivetrain drivetrain;

  private final double spinUpTime;
  private final double shootTime;
  private final double driveTime;
  private double startTime;
  private boolean finished;

  public AutoStuff(final Shooter shooter, final Serializer serializer, final BasicDifferentialDrivetrain drivetrain) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    this.shooter = shooter;
    this.serializer = serializer;
    this.drivetrain = drivetrain;
    spinUpTime = 2;
    shootTime = 4;
    driveTime = 15;
    finished = false;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    serializer.setAuto(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;

//    if (elapsedTime < driveTime) {
//      drivetrain.curvatureDrive(-0.25, 0);
//    } else {
//      drivetrain.curvatureDrive(0.0, 0.0);
//    }

    if (elapsedTime < spinUpTime){
//      shooter.setPowerRaw(shooterConstants.frontPower);
    } else if (elapsedTime < shootTime){
//      shooter.setPowerRaw(shooterConstants.frontPower);
//      serializer.setPowerRaw(SerializerConstants.enablePower);
    } else if (elapsedTime < driveTime){
//      shooter.setPowerRaw(0);
      serializer.setPowerRaw(0);
      drivetrain.curvatureDrive(-0.25, 0);
    } else {
//      shooter.setPowerRaw(0);
      serializer.setPowerRaw(0);
      serializer.setAuto(false);
      drivetrain.curvatureDrive(0, 0);
      finished = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    shooter.disable();
    serializer.setPowerRaw(0);
    serializer.setAuto(false);
    drivetrain.curvatureDrive(0, 0);
  }
}