// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateCommand extends CommandBase {
  /** Creates a new ArmRotateManualCommand. */
  ArmSubsystem m_ArmSubsystem;
  char direction;
  boolean directionUp;
  public ArmRotateCommand(ArmSubsystem m_ArmSubsystem, boolean directionUp){
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.directionUp = directionUp;
    addRequirements(this.m_ArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(directionUp){
      direction = 'u';
    }
    else{
      direction = 'd';
    }

    if(direction == 'u'){
      m_ArmSubsystem.autoRotateArm(Constants.INTAKEROTATEUPSPEED);
    }
    else if(direction == 'd'){
      m_ArmSubsystem.autoRotateArm(Constants.INTAKEROTATEDOWNSPEED);
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
