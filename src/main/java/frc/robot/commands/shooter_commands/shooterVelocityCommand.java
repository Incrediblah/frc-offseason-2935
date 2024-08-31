// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter_commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class shooterVelocityCommand extends Command {

  private ShooterSubsystem SHOOTER_SUBSYSTEM; 

  private double topVelocity; 
  private double bottomVelocity; 

  private SlewRateLimiter topLimiter = new SlewRateLimiter(ShooterConstants.topSlewLimit); 
  private SlewRateLimiter bottomLimiter = new SlewRateLimiter(ShooterConstants.bottomSlewLimit); 



  /** Creates a new shooterVelocityCommand. */
  public shooterVelocityCommand(ShooterSubsystem shooter, double topMotorVelocity, double bottomMotorVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.SHOOTER_SUBSYSTEM = shooter; 
    this.topVelocity = topMotorVelocity; 
    this.bottomVelocity = bottomMotorVelocity; 

    addRequirements(SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SHOOTER_SUBSYSTEM.setShooterVelocityMode();
    SHOOTER_SUBSYSTEM.setRampRate(ShooterConstants.shooterRampRate);
    SHOOTER_SUBSYSTEM.setTopPIDF(ShooterConstants.topShooterKp, ShooterConstants.topShooterKi, ShooterConstants.topShooterKd, ShooterConstants.topShooterKFf);
    SHOOTER_SUBSYSTEM.setBottomPIDF(ShooterConstants.bottomShooterKp, ShooterConstants.bottomShooterKi, ShooterConstants.bottomShooterKd, ShooterConstants.bottomShooterKFf);

    SHOOTER_SUBSYSTEM.setTopEncoderOutputConstraints(ShooterConstants.topMin, ShooterConstants.topMax);
    SHOOTER_SUBSYSTEM.setBottomEncoderOutputConstraints(ShooterConstants.bottomMin, ShooterConstants.bottomMax);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    System.out.println("SHOOTER COMMAND!!"); 


    SHOOTER_SUBSYSTEM.setVelocityTop((topVelocity*3.0));
    SHOOTER_SUBSYSTEM.setVelocityBottom((bottomVelocity*3.0));  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SHOOTER_SUBSYSTEM.setCoastMode();
    SHOOTER_SUBSYSTEM.setShooterPowerMode();
    SHOOTER_SUBSYSTEM.setShooter(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
