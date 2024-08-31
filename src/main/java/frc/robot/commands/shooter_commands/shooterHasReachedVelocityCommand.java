// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StatusVariables;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class shooterHasReachedVelocityCommand extends Command {
  /** Creates a new shooterHasReachedVelocity. */

  private ShooterSubsystem SHOOTER_SUBSYSTEM;
  
  private double bottomTargetVelocity; 
  private double topTargetVelocity; 

  private boolean endCommand; 

  private double shooterBottomPresentVelocity; 
  private double shooterTopPresentVelocity; 

  private boolean hasReached; 

  private double initTime; 

  public shooterHasReachedVelocityCommand(ShooterSubsystem shooter, double targetTopVelocity, double targetBottomVelocity, boolean end) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.SHOOTER_SUBSYSTEM = shooter; 
    this.topTargetVelocity = targetTopVelocity; 
    this.bottomTargetVelocity = targetBottomVelocity; 
    this.endCommand = end;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterBottomPresentVelocity = 0; 
    shooterTopPresentVelocity = 0; 
    hasReached = false;
    StatusVariables.targetVelocityReached = false; 
    initTime = System.currentTimeMillis(); 
    StatusVariables.revedUpActivated = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // System.out.println("YO THIS MOFO IS RUNNING!"); 
    shooterTopPresentVelocity = SHOOTER_SUBSYSTEM.getTopShooterEncoderVelocity(); 
    shooterBottomPresentVelocity = SHOOTER_SUBSYSTEM.getBottomShooterEncoderVelocity(); 

    if(Math.abs(shooterTopPresentVelocity/ (topTargetVelocity)) > 0.50 && Math.abs(shooterBottomPresentVelocity/ (bottomTargetVelocity)) > 0.50){
      hasReached = true; 
      StatusVariables.targetVelocityReached = true; 
    }

    else{
      hasReached = false; 
      StatusVariables.targetVelocityReached = false; 
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    StatusVariables.revedUpActivated = false;
    StatusVariables.targetVelocityReached = false; 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(endCommand == true){
      return true; 
    }

    else{
      return false; 
    }
  }
}
