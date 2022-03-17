// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbExtendSubsystem;

public class ClimbExtendCommand extends CommandBase {
  /**********************************************
    This command will turn on the climber. It just turns on the climb motor unless its hitting its associated limit switch.

    Its based on an input in the constructor. Thats it. 
  **********************************************/
  private ClimbExtendSubsystem m_ClimbExtendSubsystem;

  private boolean directionUp;
  public ClimbExtendCommand(ClimbExtendSubsystem m_ClimbExtendSubsystem, boolean directionUp) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ClimbExtendSubsystem = m_ClimbExtendSubsystem;
    this.directionUp = directionUp;
    addRequirements(m_ClimbExtendSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(directionUp){
        m_ClimbExtendSubsystem.climbExtend(Constants.CLIMB_UP_SPEED);
      }
      else{
        m_ClimbExtendSubsystem.climbExtend(Constants.CLIMB_DOWN_SPEED);
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
