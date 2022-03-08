// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonomousIntakeCommand extends CommandBase {
  /** Creates a new AutonomousIntakeCommand. */
  IntakeSubsystem m_IntakeSubsystem;
  ArmSubsystem m_ArmSubsystem;
  double timerDuration;
  Timer intakeTimer;

  public AutonomousIntakeCommand(double duration, IntakeSubsystem m_IntakeSubsystem, ArmSubsystem m_ArmSubsystem) {
      this.m_IntakeSubsystem = m_IntakeSubsystem;
      this.m_ArmSubsystem = m_ArmSubsystem;
      this.timerDuration = duration;

      addRequirements(m_IntakeSubsystem);
      addRequirements(m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      intakeTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_IntakeSubsystem.setIntakeRoller(Constants.INTAKE_ROLLER);
      m_ArmSubsystem.rotateArm(Constants.INTAKEROTATEDOWNSPEED);

      m_IntakeSubsystem.setAutoFeedWheel(Constants.FEEDSPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intakeTimer.stop();
      m_IntakeSubsystem.turnOffMotors();
      m_ArmSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intakeTimer.get() > timerDuration)
        return true;
    return false;
  }
}
