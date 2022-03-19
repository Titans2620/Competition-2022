// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbExtendSubsystem;

public class ClimbExtendDefaultCommand extends CommandBase {
/*******************************************************
By default the climb motors will be off and no action will be taken by the subsystem.
*******************************************************/

  ClimbExtendSubsystem m_ClimbExtendSubsystem;
  public ClimbExtendDefaultCommand(ClimbExtendSubsystem m_ClimbExtendSubsystem) {
    this.m_ClimbExtendSubsystem = m_ClimbExtendSubsystem;
    addRequirements(this.m_ClimbExtendSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_ClimbExtendSubsystem.stopMotors();
  }
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
