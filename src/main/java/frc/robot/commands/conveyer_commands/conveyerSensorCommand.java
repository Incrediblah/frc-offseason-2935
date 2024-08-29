package frc.robot.commands.conveyer_commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyerConstants;
import frc.robot.subsystems.ConveyerSubsystem;

public class conveyerSensorCommand extends Command {
  /** Creates a new conveySensorCommand. */

  private ConveyerSubsystem CONVEYER_SUBSYSTEM; 
  private double initTime; 
    private double conveyerSpeed; 

  public conveyerSensorCommand(ConveyerSubsystem conveyer, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.CONVEYER_SUBSYSTEM = conveyer; 
    this.conveyerSpeed = speed; 
    addRequirements(CONVEYER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = System.currentTimeMillis(); 
    CONVEYER_SUBSYSTEM.setConveyerPowerMode();
    System.out.println("SENSOR COMMAND RUNNING!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CONVEYER_SUBSYSTEM.setConveyer(conveyerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SENSOR COMMAND ENDED");
    CONVEYER_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(CONVEYER_SUBSYSTEM.getConveyerSwitchOneValue() == false||CONVEYER_SUBSYSTEM.getConveyerSwitchTwoValue()==false){
      return true; 
    }
    else{
      return false; 
    }
  }
}
