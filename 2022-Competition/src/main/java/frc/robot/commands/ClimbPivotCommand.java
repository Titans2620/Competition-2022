// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbExtendSubsystem;
import frc.robot.subsystems.ClimbPivotSubsystem;

public class ClimbPivotCommand extends CommandBase {

  ClimbPivotSubsystem m_ClimbPivotSubsystem;
  boolean isforward;
  public ClimbPivotCommand(ClimbPivotSubsystem m_ClimbPivotSubsystem, boolean isforward) {
    this.m_ClimbPivotSubsystem = m_ClimbPivotSubsystem;
    this.isforward = isforward;
    addRequirements(m_ClimbPivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(isforward){
          m_ClimbPivotSubsystem.autoPivotClimb(Constants.CLIMB_PIVOT_FORWARD_SPEED);
      }
      else{
          m_ClimbPivotSubsystem.autoPivotClimb(Constants.CLIMB_PIVOT_BACK_SPEED);
      }
    
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
