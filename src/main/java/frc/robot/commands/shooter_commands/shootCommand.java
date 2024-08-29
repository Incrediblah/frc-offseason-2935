// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class shootCommand extends Command {

  private ShooterSubsystem shooterSubsystem; 
  
  private double topSpeed; 
  private double bottomSpeed; 



  /** Creates a new intakeCommand. */
  public shootCommand(ShooterSubsystem shoot, double topSpeed, double bottomSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shoot; 
    this.topSpeed = topSpeed; 
    this.bottomSpeed = bottomSpeed; 
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setCoastMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setShooter(topSpeed, bottomSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
