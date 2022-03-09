// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateManualCommand extends CommandBase {
  /** Creates a new ArmRotateManualCommand. */
  ArmSubsystem m_ArmSubsystem;
  boolean directionUp;
  public ArmRotateManualCommand(ArmSubsystem m_ArmSubsystem, boolean directionUp) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.directionUp = directionUp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(directionUp){
      m_ArmSubsystem.rotateArm(Constants.INTAKEROTATEUPSPEED);
    }
    else if(!directionUp){
      m_ArmSubsystem.rotateArm(Constants.INTAKEROTATEDOWNSPEED);
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
