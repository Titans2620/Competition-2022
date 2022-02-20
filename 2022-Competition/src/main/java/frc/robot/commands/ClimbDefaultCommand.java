// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDefaultCommand extends CommandBase {
/*******************************************************
By default the climb motors will be off and no action will be taken by the subsystem.
*******************************************************/

  ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  public ClimbDefaultCommand(ClimbSubsystem m_ClimbSubsystem) {
    addRequirements(this.m_ClimbSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
