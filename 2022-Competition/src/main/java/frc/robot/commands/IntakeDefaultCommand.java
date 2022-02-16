// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends CommandBase {
  /********************************************************
  The default intake command should have all motors off and no action should be taken by the subsystem.
  ***********************************************************/

  IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  public IntakeDefaultCommand(IntakeSubsystem intakeSubsystem) {
      m_intakeSubsystem = intakeSubsystem;
      addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
      m_intakeSubsystem.turnOffMotors();
  }

  @Override
  public void end(boolean interrupted) {
      m_intakeSubsystem.turnOffMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
