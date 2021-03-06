// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class IntakeManualCommand extends CommandBase {
  /**************************************************
   This will only be run when the infeed button is pressed in manual mode.
   
   This will function identically to the infeed command except it will run without sensors.
   *************************************************/
  IntakeSubsystem m_IntakeSubsystem;
  public IntakeManualCommand(IntakeSubsystem m_IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LEDSubsystem.setState("SolidYellow", 5);
    m_IntakeSubsystem.setFeedWheel(Constants.FEEDSPEED);
    m_IntakeSubsystem.setIntakeRoller(Constants.INTAKESPEED);

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
