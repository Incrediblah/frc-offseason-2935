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

  private PhotonCamera aprilTagCamera; 
  private PhotonCamera noteCamera; 

  private PhotonTrackedTarget bestTarget = null;


  public PhotonSubsystem() {
    aprilTagCamera = new PhotonCamera(photonVisionConstants.tagCameraName); 
    noteCamera = new PhotonCamera(photonVisionConstants.intakeCameraName); 

  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("PHOTON TAG ACTIVE", photonTagActive()); 
    SmartDashboard.putBoolean("PHOTON HAS TAG", photonTagHasTargets()); 


    SmartDashboard.putBoolean("PHOTON NOTE ACTIVE", photonNoteActive()); 
    SmartDashboard.putBoolean("PHOTON HAS NOTE", photonNoteHasTargets()); 

  }


  // PHOTON TAG STUFF
  public boolean photonTagActive(){

    if(aprilTagCamera.isConnected()){
      return true; 
    } else{
      return false; 
    }  
  }

  public PhotonPipelineResult photonTagResult(){
    return aprilTagCamera.getLatestResult(); 
  }

  public boolean photonTagHasTargets(){
    if(photonTagResult().hasTargets()){
      return true; 
    }

    else{
      return false; 
    }
  }

  public PhotonTrackedTarget getBestTag(int id){
    if(photonTagResult().hasTargets()){

            // Loop through detected targets to find the AprilTag with the desired ID
      for (var target : photonTagResult().getTargets()) {
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

  public double getTagYaw(){
    if(photonTagHasTargets()){
      return photonTagResult().getBestTarget().getYaw(); 
    }
    else{
      return 0; 
    }
  }

  public double getTagPitch(){
    if(photonTagHasTargets()){
      return photonTagResult().getBestTarget().getPitch(); 
    }else{
      return 0; 
    }
    
  }

  public double getBestTagTargetYaw(){
    if(bestTarget != null){
      return bestTarget.getYaw(); 
    }else{
      return 0; 
    }
  }

  public double getBestTagTargetPitch(){
    if(bestTarget != null){
      return bestTarget.getPitch(); 
    }else{
      return 0; 
    }
  }

  // --------------- PHOTON NOTE STUFF

   public boolean photonNoteActive(){

    if(noteCamera.isConnected()){
        return true; 
      } else{
        return false; 
      }  
  }

  public PhotonPipelineResult photonNoteResult(){
    return noteCamera.getLatestResult(); 
  }

  public boolean photonNoteHasTargets(){
    if(photonNoteResult().hasTargets()){
      return true; 
    }

    else{
      return false; 
    }
  }

  public double getNoteYaw(){
    if(photonNoteHasTargets()){
      return photonNoteResult().getBestTarget().getYaw(); 
    }
    else{
      return 0; 
    }
  }

  public double getNotePitch(){
    if(photonNoteHasTargets()){
      return photonNoteResult().getBestTarget().getPitch(); 
    }else{
      return 0; 
    }
    
  }

  public double getBestNoteTargetYaw(){
    if(bestTarget != null){
      return bestTarget.getYaw(); 
    }else{
      return 0; 
    }
  }

  public double getBestNoteTargetPitch(){
    if(bestTarget != null){
      return bestTarget.getPitch(); 
    }else{
      return 0; 
    }
  }
}
