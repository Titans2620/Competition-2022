// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmRotateManualCommand extends CommandBase {
  /** Creates a new ArmRotateManualCommand. */
  ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  boolean up;
  boolean down;
  public ArmRotateManualCommand(ArmSubsystem m_ArmSubsystem, boolean up, boolean down){
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.up = up;
    this.down = down;
    addRequirements(this.m_ArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(up){
      m_ArmSubsystem.rotateArmUp(Constants.INTAKEROTATESPEED);
    }
    else if(down){
      m_ArmSubsystem.rotateArmDown(-Constants.INTAKEROTATESPEED);
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
