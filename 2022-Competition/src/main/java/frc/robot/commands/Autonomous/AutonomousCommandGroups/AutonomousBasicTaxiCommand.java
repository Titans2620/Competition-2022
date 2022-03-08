// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.AutonomousCommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.AutonomousBasicMovementCommand;
import frc.robot.commands.Autonomous.AutonomousIntakeCommand;
import frc.robot.commands.Autonomous.AutonomousShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousBasicTaxiCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousBasicTaxiCommand. */
  DriveSubsystem m_DriveSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  ArmSubsystem m_ArmSubsystem;

  public AutonomousBasicTaxiCommand(DriveSubsystem m_DriveSubsystem, IntakeSubsystem m_IntakeSubsystem, ShooterSubsystem m_ShooterSubsystem, ArmSubsystem m_ArmSubsystem, String alliance) {
    
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.m_ArmSubsystem = m_ArmSubsystem;

    addCommands(
        new ParallelCommandGroup(
            new AutonomousBasicMovementCommand(0.25, 0.25, 0, 3, m_DriveSubsystem), 
            new AutonomousIntakeCommand(3, m_IntakeSubsystem, m_ArmSubsystem)
        ),
        new AutonomousBasicMovementCommand(0, 0, 180, 3, m_DriveSubsystem),
        new AutonomousShootCommand(m_DriveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, 5, alliance)
    );
  }
}
