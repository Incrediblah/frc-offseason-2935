// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.photonCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;

public class photonAutoIntake extends Command {

    private PhotonSubsystem PHOTON_SUBSYSTEM;
  private DriveSubsystem DRIVE_SUBSYSTEM;  

  private double bestTargetYaw; 
  private double bestTargetPitch; 

  private double driveSpeed = 0; 

  private boolean endCommand = false; 
  private boolean inRange = false; 

  private PIDController translateController;
  private PIDController driveController;
  private double translateSpeed; 

  /** Creates a new photonAutoIntake. */
  public photonAutoIntake(PhotonSubsystem photon, DriveSubsystem drive, boolean end) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.PHOTON_SUBSYSTEM = photon; 
    this.DRIVE_SUBSYSTEM = drive; 
    this.endCommand = end; 

    this.translateController = new PIDController(0.025, 0, 0); 
    this.driveController = new PIDController(0.025, 0, 0); 


    addRequirements(PHOTON_SUBSYSTEM);
    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    translateController.reset();
    driveController.reset();
    translateSpeed = 0; 
    driveSpeed = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println("COMAND IS ACTUALLY RUNNING MULTIPLE TIMES!"); 
    if(PHOTON_SUBSYSTEM.photonNoteHasTargets()){
      bestTargetYaw = PHOTON_SUBSYSTEM.getNoteYaw(); 
      bestTargetPitch = PHOTON_SUBSYSTEM.getNotePitch(); 
    }else{
      bestTargetYaw = 0; 
      bestTargetPitch = 0; 
    }

    translateSpeed = translateController.calculate(bestTargetYaw, 0); 
    driveSpeed = -driveController.calculate(bestTargetPitch, -15); 
    
    if(bestTargetYaw < -1){ 
      inRange = false; 
    }else if(bestTargetYaw > 1){
      inRange = false; 
    }else{
      inRange = true; 
      translateSpeed = 0; 
    }

    DRIVE_SUBSYSTEM.drive(0, translateSpeed, 0, false, false);
    // DRIVE_SUBSYSTEM.drive(translateSpeed, 0, 0, false, false);

    SmartDashboard.putNumber("best taregt yaw FOR note", bestTargetYaw); 
    SmartDashboard.putBoolean("in_range", inRange); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSpeed = 0; 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(endCommand){
      return true; 
    }else{
      return false; 
    }
  }
}
