// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;

    // SPEED LIMITERS 
    public static final double driveSpeedLimiter = 0.25;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class IntakeConstants{
    public static final int intakeMotorCANId = 9; 
    public static final double intakeSpeed = 0.25; // amps

    public static final int neoRPM = 5676; 


    public static final double intakeVelocity = neoRPM * 0.25;
    
    

     // INTAKE
     public static double intakeKp = 6e-5; 
     public static double intakeKi = 0.0000; 
     public static double intakeKd = 0; 
     public static double intakeKIz = 6e-5; 
     public static double intakeKFf = 0.000015;
 
     // SHOOTER MOTOR OUTPUT CONSTRAINTS
     public static double intakeMax = 1; 
     public static double intakeMin = -1; 
 
     // SHOOTER MOTOR SLEW RATE LIMITERS 
     public static double intakeSlewLimit = 2; 

     // SHOOTER MOTOR RAMP RATE 
     public static double intakeRampRate = 0.25;
  }

  public static final class ConveyerConstants{
    public static final int conveyerMotorCANId = 10; 
    public static final double conveyerSpeed = 0.5; // amps

    public static final int neoRPM = 5676; 

    public static final double conveyerVelocity = neoRPM * 0.75; 
   
   
   // Conveyer
     public static double conveyerKp = 6e-5; 
     public static double conveyerKi = 0.0000; 
     public static double conveyerKd = 0; 
     public static double conveyerKIz = 6e-5; 
     public static double conveyerKFf = 0.000015;
 
     // SHOOTER MOTOR OUTPUT CONSTRAINTS
     public static double conveyerMax = 1; 
     public static double conveyerMin = -1; 
 
     // SHOOTER MOTOR SLEW RATE LIMITERS 
     public static double conveyerSlewLimit = 2; 

     // SHOOTER MOTOR RAMP RATE 
     public static double conveyerRampRate = 0.25;
  }

    
   
  


  public static final class ShooterConstants{

        // MOTOR IDS FOR SHOOTER MOTORS 
    public static final int topShooterMotorId = 11; 
    public static final int bottomShooterMotorId = 12; 

    public static final int neoVortexRPM = 6784; 

    public static final double podiumTopSpeed = 1; 
    public static final double podiumBottomSpeed = 0.5; 


    public static final double podiumTopVelocity = neoVortexRPM; 
    public static final double podiumBottomVelocity = neoVortexRPM * 0.425; 

    public static final double subwooferTopVelocity = neoVortexRPM * 0.25; 
    public static final double subwooferBottomVelocity = neoVortexRPM; 


    public static final double autolineTopVelocity = neoVortexRPM * 0.8 ; 
    public static final double autolineBottomVelocity = neoVortexRPM * 0.7; 

    public static final double ampTopVelocity = neoVortexRPM * 0.10 ; 
    public static final double ampBottomVelocity = neoVortexRPM * 1; 





    public static final double shooterIntakeTopSpeed = -0.25; 
    public static final double shooterIntakeBottomSpeed = -0.25; 

    public static final double conveyerSpeed = -0.25; 
   
    
    
    

    // SPEED TO KEEP NOTE IN DURING CENTRING 
    public static final double shooterHoldInSpeed = neoVortexRPM * -0.1; 

     // TOP SHOOTER MOTOR PID 
     public static double topShooterKp = 6e-5; 
     public static double topShooterKi = 0.0000; 
     public static double topShooterKd = 0; 
     public static double topShooterKIz = 6e-5; 
     public static double topShooterKFf = 0.000015;
 
     // BOTTOM SHOOTER MOTOR  PID
     public static double bottomShooterKp = 6e-5; 
     public static double bottomShooterKi = 0.0000; 
     public static double bottomShooterKd = 0; 
     public static double bottomShooterKIz = 6e-5; 
     public static double bottomShooterKFf = 0.000015;
 
     // SHOOTER MOTOR OUTPUT CONSTRAINTS
     public static double topMax = 1; 
     public static double topMin = -1; 
 
     public static double bottomMax = 1; 
     public static double bottomMin = -1; 
 
     // SHOOTER MOTOR SLEW RATE LIMITERS 
     public static double topSlewLimit = 2; 
     public static double bottomSlewLimit = 2; 
 
     // SHOOTER MOTOR RAMP RATE 
     public static double shooterRampRate = 0.25;
    
  }


  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;

     // CONTROLLER PORTS 
     public static final int primaryControllerPort = 0; 
     public static final int secondaryControllerPort = 1; 
 
     // BUTTON MAPPING 
     public static final int BUTTON_A_PORT = 1;
     public static final int BUTTON_B_PORT = 2;
     public static final int BUTTON_X_PORT = 3;
     public static final int BUTTON_Y_PORT = 4;
 
     public static final int BUTTON_RB_PORT = 6;
     public static final int BUTTON_LB_PORT = 5;
 
     public static final int BUTTON_START = 8;
     public static final int BUTTON_RIGHT_JOYSTICK_PORT = 9;
     public static final int BUTTON_LEFT_JOYSTICK_PORT = 10;
   
     public static final int driveJoystickAxis = 1; 
     public static final int turnJoystickAxis = 4; 
 
     public static final int rightTriggerAxis = 3; 
     public static final int leftTriggerAxis = 2; 
 
     // TRIGGER ACTIVATION THRESHOLD 
     public static final double triggerThreshold = 0.5; 
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }
}