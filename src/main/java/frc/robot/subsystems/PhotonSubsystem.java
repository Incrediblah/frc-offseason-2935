// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StatusVariables;
import frc.robot.Constants.photonVisionConstants;

public class PhotonSubsystem extends SubsystemBase {
  /** Creates a new PhotonSubsystem. */

  private PhotonCamera photonCamera; 
  private PhotonTrackedTarget bestTarget = null;


  public PhotonSubsystem() {
    photonCamera = new PhotonCamera(photonVisionConstants.cameraName); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("target id", StatusVariables.targetID); 

    SmartDashboard.putBoolean("PHOTON ACTIVE", photonActive()); 
    SmartDashboard.putBoolean("PHOTON HAS TARGET", photonHasTargets()); 
  }

  public boolean photonActive(){

    if(photonCamera.isConnected()){
      return true; 
    } else{
      return false; 
    }  
  }

  public PhotonPipelineResult photonResult(){
    return photonCamera.getLatestResult(); 
  }

  public boolean photonHasTargets(){
    if(photonResult().hasTargets()){
      return true; 
    }

    else{
      return false; 
    }
  }

  public PhotonTrackedTarget getBestTarget(int id){
    if(photonResult().hasTargets()){

            // Loop through detected targets to find the AprilTag with the desired ID
      for (var target : photonResult().getTargets()) {
          if (target.getFiducialId() == id) {
              bestTarget = target;
              break; // Stop searching once we've found our target
          }
      }

      return bestTarget; 
    }

    else{
      return null; 
    }


  }

  public double getYaw(){
    if(photonHasTargets()){
      return photonResult().getBestTarget().getYaw(); 
    }
    else{
      return 0; 
    }
  }

  public double getPitch(){
    if(photonHasTargets()){
      return photonResult().getBestTarget().getPitch(); 
    }else{
      return 0; 
    }
    
  }

  public double getBestTargetYaw(){
    if(bestTarget != null){
      return bestTarget.getYaw(); 
    }else{
      return 0; 
    }
  }

  public double getBestTargetPitch(){
    if(bestTarget != null){
      return bestTarget.getPitch(); 
    }else{
      return 0; 
    }
  }
}
