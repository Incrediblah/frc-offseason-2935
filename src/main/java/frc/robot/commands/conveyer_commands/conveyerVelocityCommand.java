
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.conveyer_commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyerConstants;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class conveyerVelocityCommand extends Command {

  private ConveyerSubsystem CONVEYER_SUBSYSTEM; 

  private double conveyerVelocity; 

  private SlewRateLimiter conveyerLimiter = new SlewRateLimiter(ConveyerConstants.conveyerSlewLimit); 

  /** Creates a new conveyerVelocityCommand. */
  public conveyerVelocityCommand(ConveyerSubsystem conveyer, double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.CONVEYER_SUBSYSTEM = conveyer; 
    this.conveyerVelocity = velocity;
    addRequirements(CONVEYER_SUBSYSTEM); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CONVEYER_SUBSYSTEM.setConveyerVelocityMode();
    CONVEYER_SUBSYSTEM.setRampRate(ConveyerConstants.conveyerRampRate);
    CONVEYER_SUBSYSTEM.setConveyerPIDF(ConveyerConstants.conveyerKp, ConveyerConstants.conveyerKi, ConveyerConstants.conveyerKd, ConveyerConstants.conveyerKFf);

    CONVEYER_SUBSYSTEM.setConveyerEncoderOutputConstraints(ConveyerConstants.conveyerMin, ConveyerConstants.conveyerMax);
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CONVEYER_SUBSYSTEM.setVelocityConveyer((conveyerVelocity * 3)); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CONVEYER_SUBSYSTEM.setCoastMode();
    CONVEYER_SUBSYSTEM.setShooterPowerMode();
    CONVEYER_SUBSYSTEM.setConveyer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
