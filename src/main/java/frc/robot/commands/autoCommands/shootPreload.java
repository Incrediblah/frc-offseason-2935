// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ConveyerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.delayCommand;
import frc.robot.commands.conveyer_commands.conveyerCommand;
import frc.robot.commands.shooter_commands.shooterVelocityCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootPreload extends SequentialCommandGroup {
  /** Creates a new shootPreload. */
  public shootPreload(ShooterSubsystem shooter, ConveyerSubsystem conveyer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ParallelRaceGroup(
         new shooterVelocityCommand(shooter, ShooterConstants.subwooferTopVelocity, ShooterConstants.subwooferBottomVelocity), 

         new SequentialCommandGroup(
          new delayCommand(3000), 

          new ParallelDeadlineGroup(
              new delayCommand(2000), 
              new conveyerCommand(conveyer, ConveyerConstants.conveyerVelocity) 
          )
       

         )
      )

    );
  }
}
