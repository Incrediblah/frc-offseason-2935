// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ConveyerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.shooter_commands.shootCommand;
import frc.robot.commands.conveyer_commands.conveyerCommand;
import frc.robot.commands.conveyer_commands.conveyerSensorCommand;
//import frc.robot.commands.conveyer_commands.conveyerSensorCommand
import frc.robot.commands.conveyer_commands.conveyerVelocityCommand;
import frc.robot.commands.intake_Command.intakeCommand;
import frc.robot.commands.intake_Command.intakeVelocityCommand;
import frc.robot.commands.shooter_commands.shooterVelocityCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import javax.sound.sampled.Port;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(); 

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ConveyerSubsystem m_conveyerSubsystem = new ConveyerSubsystem(); 


  public final SendableChooser<Command> autoChooser;

    // input    

  private final Joystick joystick = new Joystick(OIConstants.primaryControllerPort); 
  private final Joystick joystickSecondary = new Joystick(OIConstants.secondaryControllerPort); 

  private final JoystickButton PRIMARY_BUTTON_LB = new JoystickButton(joystick, OIConstants.BUTTON_LB_PORT);    
  private final JoystickButton PRIMARY_BUTTON_RB = new JoystickButton(joystick, OIConstants.BUTTON_RB_PORT);    
  private final JoystickButton PRIMARY_BUTTON_A = new JoystickButton(joystick, OIConstants.BUTTON_A_PORT);    
  private final JoystickButton PRIMARY_BUTTON_B = new JoystickButton(joystick, OIConstants.BUTTON_B_PORT);    
  
  private final JoystickButton SECONDARY_BUTTON_LB = new JoystickButton(joystickSecondary, OIConstants.BUTTON_LB_PORT);    
  private final JoystickButton SECONDARY_BUTTON_RB = new JoystickButton(joystickSecondary, OIConstants.BUTTON_RB_PORT);    
  private final JoystickButton SECONDARY_BUTTON_A = new JoystickButton(joystickSecondary, OIConstants.BUTTON_A_PORT);    
  private final JoystickButton SECONDARY_BUTTON_B = new JoystickButton(joystickSecondary, OIConstants.BUTTON_B_PORT);    
  private final JoystickButton SECONDARY_BUTTON_Y = new JoystickButton(joystickSecondary, OIConstants.BUTTON_Y_PORT);    
  private final JoystickButton SECONDARY_BUTTON_X = new JoystickButton(joystickSecondary, OIConstants.BUTTON_X_PORT);    

  
  private final CommandGenericHID controllerPrimary = new CommandGenericHID(OIConstants.primaryControllerPort);  
  private final CommandGenericHID controllerSecondary = new CommandGenericHID(OIConstants.secondaryControllerPort);  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

        m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(joystick.getRawAxis(1)*DriveConstants.driveSpeedLimiter, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(joystick.getRawAxis(0)*DriveConstants.driveSpeedLimiter, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(joystick.getRawAxis(4)*DriveConstants.driveSpeedLimiter, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    // Configure the button bindings

    //  NamedCommands.registerCommand("AutoIntake", new AutoIntakeTimeout(m_intake, m_storage, m_pivot, m_leds));


    //Pathplanner auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser); 

    //Put the autons on the chooser and on SmartDashboard
    SmartDashboard.putData("gay", new PathPlannerAuto("gay-auto"));



    configureButtonBindings();
    defaultCommands(); 

  }



    private void defaultCommands(){
            // Configure default commands
 
    }

  private void configureButtonBindings() {

    // INTAKE
    SECONDARY_BUTTON_LB.onTrue(

        new ParallelDeadlineGroup(
             new conveyerSensorCommand(m_conveyerSubsystem, ConveyerConstants.conveyerSpeed),
            new intakeVelocityCommand(m_intakeSubsystem, IntakeConstants.intakeVelocity) 
            
           // new conveyerVelocityCommand(m_conveyerSubsystem, ConveyerConstants.conveyerVelocity)
            //new conveyerSensorCommand(m_conveyerSubsystem,ConveyerConstants.conveyerVelocity )
        )
        
    ); 

    SECONDARY_BUTTON_LB.onFalse(
        new ParallelCommandGroup(
            new intakeCommand(m_intakeSubsystem, 0), 
            new conveyerVelocityCommand(m_conveyerSubsystem, 0)
        )
    ); 

    // OUTTAKE
    SECONDARY_BUTTON_RB.onTrue(
        new ParallelCommandGroup(
            new intakeVelocityCommand(m_intakeSubsystem, -IntakeConstants.intakeVelocity), 
           // new conveyerVelocCommand(m_conveyerSubsystem, ConveyerConstants.conveyerSpeed) 
            new conveyerVelocityCommand(m_conveyerSubsystem, -ConveyerConstants.conveyerVelocity)
        )

    ); 

    SECONDARY_BUTTON_RB.onFalse(
        new ParallelCommandGroup(
             
            // new intakeCommand(m_intakeSubsystem, 0), 
            new conveyerVelocityCommand(m_conveyerSubsystem, 0)
        )
    ); 

    // SHOOTER podium speed 

    SECONDARY_BUTTON_A.onTrue(
        new shooterVelocityCommand(m_shooterSubsystem, ShooterConstants.podiumTopVelocity, ShooterConstants.podiumBottomVelocity)
    ); 

    SECONDARY_BUTTON_A.onFalse(
        new shooterVelocityCommand(m_shooterSubsystem, 0, 0) 
    ); 

    // shooter subwoofer speed 

    SECONDARY_BUTTON_Y.onTrue(
        new shooterVelocityCommand(m_shooterSubsystem, ShooterConstants.subwooferTopVelocity, ShooterConstants.subwooferBottomVelocity)
    ); 

    SECONDARY_BUTTON_Y.onFalse(
        new shooterVelocityCommand(m_shooterSubsystem, 0, 0) 
    ); 
    
     // shooter autoline speed
 
    SECONDARY_BUTTON_X.onTrue(
        new shooterVelocityCommand(m_shooterSubsystem, ShooterConstants.autolineTopVelocity, ShooterConstants.autolineBottomVelocity)
    ); 

    SECONDARY_BUTTON_X.onFalse(
        new shooterVelocityCommand(m_shooterSubsystem, 0, 0) 
    ); 
    // amp shot
    SECONDARY_BUTTON_B.onTrue(
        new shooterVelocityCommand(m_shooterSubsystem, ShooterConstants.ampTopVelocity, ShooterConstants.ampBottomVelocity)
    ); 

    SECONDARY_BUTTON_B.onFalse(
        new shooterVelocityCommand(m_shooterSubsystem, 0, 0) 
    );


    controllerSecondary.axisGreaterThan(OIConstants.leftTriggerAxis, OIConstants.triggerThreshold).toggleOnFalse(new conveyerVelocityCommand(m_conveyerSubsystem, 0));
    controllerSecondary.axisGreaterThan(OIConstants.leftTriggerAxis, OIConstants.triggerThreshold).toggleOnTrue(new conveyerVelocityCommand(m_conveyerSubsystem, ConveyerConstants.conveyerVelocity));
  }
        /*shoot buttons
        amp=B
        autoline =X
        subwoofer=Y
        podium=A

   
   

   


  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
        // Load the path you want to follow using its name in the GUI

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return autoChooser.getSelected(); 
    }
}
