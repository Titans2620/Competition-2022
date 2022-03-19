// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbExtendSubsystem;

public class ClimbExtendSingleCommand extends CommandBase {
  /** Creates a new ClimbExtendSingleCommand. */
  ClimbExtendSubsystem m_ClimbExtendSubsystem;
  boolean directionUp;
  boolean runLeft;
  public ClimbExtendSingleCommand(ClimbExtendSubsystem m_ClimbExtendSubsystem, boolean directionUp, boolean runLeft) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ClimbExtendSubsystem = m_ClimbExtendSubsystem;
    this.directionUp = directionUp;
    this.runLeft = runLeft;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(runLeft && directionUp){
      m_ClimbExtendSubsystem.climbExtendLeft(Constants.CLIMB_UP_SPEED); //Left Up
    }
    else if(runLeft && !directionUp){
      m_ClimbExtendSubsystem.climbExtendLeft(Constants.CLIMB_DOWN_SPEED); //Left Down
    }
    else if(!runLeft && directionUp){
      m_ClimbExtendSubsystem.climbExtendRight(Constants.CLIMB_UP_SPEED); //Right Up
    }
    else if(!runLeft && !directionUp){
      m_ClimbExtendSubsystem.climbExtendRight(Constants.CLIMB_DOWN_SPEED); //Right Down
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
