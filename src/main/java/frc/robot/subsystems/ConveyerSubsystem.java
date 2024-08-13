// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyerConstants;


public class ConveyerSubsystem extends SubsystemBase {


  private final String CONVEYER_PREFIX = "SmartDashboard/Conveyer"; 

  private CANSparkMax conveyer_motor = new CANSparkMax(ConveyerConstants.conveyerMotorCANId, MotorType.kBrushless); 
  private RelativeEncoder conveyer_encoder = conveyer_motor.getEncoder();

  /** Creates a new ConveyerSubsystem. */
  public ConveyerSubsystem() {
     // RESTORE SETTINGS 
     conveyer_motor.restoreFactoryDefaults(); 
     // INVERT 
     conveyer_motor.setInverted(true);
     // SET POSITION TO 0 
     conveyer_encoder.setPosition(0);
     // LOAD CONFIG 
     conveyer_motor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(CONVEYER_PREFIX + "conveyer encoder", getconveyer_encoderPosition()); 
     
     // PRINT conveyer VELOCITY   
     SmartDashboard.putNumber(CONVEYER_PREFIX + "conveyer velocity", getconveyer_encoderVelocity()); 
 
     // PRINT MOTOR CURRENT USAGE 
     SmartDashboard.putNumber(CONVEYER_PREFIX + "conveyer current", getconveyer_EncoderCurrent()); 

  }

      // SET TO BRAKE MODE 
  public void setBrakeMode(){
    conveyer_motor.setIdleMode(IdleMode.kBrake); 
  }

  // SET TO COAST MODE 
  public void setCoastMode(){
    conveyer_motor.setIdleMode(IdleMode.kCoast); 
  }

  // RESET ENCODER
  public void resetEncoders(){
    conveyer_encoder.setPosition(0);  
  }

  public double getconveyer_encoderPosition(){
    return conveyer_encoder.getPosition(); 
  }

  public double getconveyer_encoderVelocity(){
    return conveyer_encoder.getVelocity();     
  }


  public double getconveyer_EncoderCurrent(){
    return conveyer_motor.getOutputCurrent(); 
  }

  // SET conveyer 
  public void setconveyer(double conveyerSpeed){
    conveyer_motor.set(conveyerSpeed);
  }
  public void setConveyerPIDF(double p, double i, double d, double f){
    conveyer_motor.getPIDController().setP(p); 
    conveyer_motor.getPIDController().setI(i); 
    conveyer_motor.getPIDController().setD(d); 
    conveyer_motor.getPIDController().setFF(f); 
  }



  public void setConveyerEncoderOutputConstraints(double min, double max){
    conveyer_motor.getPIDController().setOutputRange(min, max); 
  }



  public void setConveyerVelocityMode(){
    conveyer_motor.getPIDController().setReference(0, ControlType.kVelocity); 
  }

  public void setShooterPowerMode(){
    conveyer_motor.getPIDController().setReference(0, ControlType.kCurrent); 
  }

  public void setVelocityConveyer(double topVelocity){
    conveyer_motor.getPIDController().setReference(topVelocity, ControlType.kVelocity); 
  }


  public void setRampRate(double ramp){
    conveyer_motor.setClosedLoopRampRate(ramp);
  }

  public void setConveyer(double conveyerSpeed){
    conveyer_motor.set(conveyerSpeed);
  }

// STOP conveyer 
  public void stop(){
    conveyer_motor.stopMotor();

 }

}



  


  