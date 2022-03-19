// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeOutfeedCommand extends CommandBase {
  /** Creates a new IntakeOutfeedCommand. */
  IntakeSubsystem m_IntakeSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  public IntakeOutfeedCommand(IntakeSubsystem m_IntakeSubsystem, ShooterSubsystem m_ShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.setFeedWheel(-Constants.FEEDSPEED);
    m_ShooterSubsystem.setShooter(-Constants.SHOOTERSPEED / 2);
    addRequirements(m_ShooterSubsystem, m_IntakeSubsystem);
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
