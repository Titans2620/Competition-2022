// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterDefaultCommand extends CommandBase {
  /********************************************************
  By default the shooter motor should be off and no action should be taken by this subsystem.
  ***********************************************************/

  ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  public ShooterDefaultCommand(ShooterSubsystem m_shooterSubsystem) {
    this.m_ShooterSubsystem = m_shooterSubsystem;
    addRequirements(m_ShooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_ShooterSubsystem.stopShooter();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
