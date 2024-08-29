// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.photonCommands;

import java.io.ObjectInputFilter.Status;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StatusVariables;
import frc.robot.Constants.photonVisionConstants;
import frc.robot.commands.shooter_commands.shooterVelocityCommand;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class photonAdjustShooter extends Command {
  /** Creates a new photonAdjustShooter. */

  private PhotonSubsystem PHOTON_SUBSYSTEM; 
  private ShooterSubsystem SHOOTER_SUBSYSTEM; 
  private boolean endCommand; 

  private double bestTargetPitch = 0; 

  public photonAdjustShooter(PhotonSubsystem photon, ShooterSubsystem shooter,  boolean end) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.PHOTON_SUBSYSTEM = photon; 
    this.SHOOTER_SUBSYSTEM = shooter; 
    this.endCommand = end; 
    addRequirements(PHOTON_SUBSYSTEM);
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

    if(PHOTON_SUBSYSTEM.photonHasTargets()){
      bestTargetPitch = PHOTON_SUBSYSTEM.getPitch(); 
    }else{; 
      bestTargetPitch = 0;  
    }

    // double range =  PhotonUtils.calculateDistanceToTargetMeters(
       
    //         photonVisionConstants.cameraHeight,
    //         photonVisionConstants.speakerHeight,
    //         photonVisionConstants.cameraMountAngle,
    //         Units.degreesToRadians(bestTargetPitch));

    
    SHOOTER_SUBSYSTEM.setVelocityTop(topMotorVelocity(bestTargetPitch) * 3); 
    SHOOTER_SUBSYSTEM.setVelocityBottom(bottomMotorVelocity(bestTargetPitch) * 3); 
    // bottomMotorVelocity(bestTargetPitch); 
    
    SmartDashboard.putNumber("calc top", StatusVariables.topCalc); 
    SmartDashboard.putNumber("calc bottom", StatusVariables.bottomCalc); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SHOOTER_SUBSYSTEM.setShooterPowerMode();
    SHOOTER_SUBSYSTEM.setShooter(0, 0);
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

  public double topMotorVelocity(double pitch){  
    StatusVariables.topCalc = (-0.0397*pitch+0.7929) * ShooterConstants.neoVortexRPM; 
    return((-0.0397*pitch+0.7929) * ShooterConstants.neoVortexRPM); 
  }

  public double bottomMotorVelocity(double pitch){
    StatusVariables.bottomCalc = (0.0331*pitch+0.5059) * ShooterConstants.neoVortexRPM; 
    return((0.0331*pitch+0.5059) * ShooterConstants.neoVortexRPM); 
  }

}
