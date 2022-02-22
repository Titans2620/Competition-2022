// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeEjectCommand extends CommandBase {
  /*********************************************
  This command will run if a ball is stuck or we otherwise need to run the infeed backwards. 

  It will function identically to the infeed command except it will not care about sensors and the feed wheel and roller will be moving in the opposite direction.
  ***********************************************/
  IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public IntakeEjectCommand(IntakeSubsystem m_intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intakeSubsystem = m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
