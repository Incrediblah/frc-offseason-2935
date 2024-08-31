// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.ObjectInputFilter.Status;

import org.opencv.ml.StatModel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.StatusVariables;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;

public class led_Command extends Command {


  private LightSubsystem LIGHT_SUBSYSTEM; 
  private int red, green, blue; 

  /** Creates a new ledCommand. */
  public led_Command(LightSubsystem light) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LIGHT_SUBSYSTEM = light; 

    addRequirements(LIGHT_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LIGHT_SUBSYSTEM.setOneColour(0, 0, 0);
    red = 0; 
    green = 0; 
    blue = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putBoolean("reved up", StatusVariables.revedUpActivated);
    SmartDashboard.putBoolean("has reached", StatusVariables.targetVelocityReached); 


    if(StatusVariables.conveyerSwitchOneStatus == false){
        if(StatusVariables.alignActivated){
          if(StatusVariables.alignedToCentre){
            red = LedConstants.greenColourCode[0]; 
            green = LedConstants.greenColourCode[1]; 
            blue = LedConstants.greenColourCode[2];  
          }

          else{
            red = LedConstants.purpleColourCode[0]; 
            green = LedConstants.purpleColourCode[1]; 
            blue = LedConstants.purpleColourCode[2];  
          }
        }

        else{
          red = LedConstants.whiteColourCode[0]; 
          green = LedConstants.whiteColourCode[1]; 
          blue = LedConstants.whiteColourCode[2]; 
        }
        
        // red = LedConstants.whiteColourCode[0]; 
        // green = LedConstants.whiteColourCode[1]; 
        // blue = LedConstants.whiteColourCode[2]; 
      

    }
    
    else{

      if(StatusVariables.allianceNumber == 0){
        red = LedConstants.redColourCode[0]; 
        green = LedConstants.redColourCode[1]; 
        blue = LedConstants.redColourCode[2]; 
      }
      
      else if(StatusVariables.allianceNumber == 1){
        red = LedConstants.blueColourCode[0]; 
        green = LedConstants.blueColourCode[1]; 
        blue = LedConstants.blueColourCode[2]; 
      }
      
      else{
        red = LedConstants.yellowColourCode[0]; 
        green = LedConstants.yellowColourCode[1]; 
        blue = LedConstants.yellowColourCode[2];
      }

    }

    LIGHT_SUBSYSTEM.setOneColour(red, blue, green); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}