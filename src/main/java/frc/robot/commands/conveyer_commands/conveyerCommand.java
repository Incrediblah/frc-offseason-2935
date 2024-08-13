// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyer_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyerSubsystem;


public class conveyerCommand extends Command {
  
  private ConveyerSubsystem CONVEYER_SUBSYSTEM; 
  
  private double conveyerSpeed; 



  /** Creates a new conveyerCommand. */
  public conveyerCommand(ConveyerSubsystem conveyer, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.CONVEYER_SUBSYSTEM = conveyer; 
    this.conveyerSpeed = speed;
    addRequirements(CONVEYER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CONVEYER_SUBSYSTEM.setconveyer(conveyerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CONVEYER_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}