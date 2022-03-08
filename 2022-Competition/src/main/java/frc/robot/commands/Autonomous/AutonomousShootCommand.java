// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousShootCommand extends CommandBase {
  /** Creates a new AutonomousShootCommand. */

  DriveSubsystem m_DriveSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  ShooterSubsystem m_ShooterSubsystem;

  Timer shootTimer = new Timer();

  String alliance;

  Double duration;

  public AutonomousShootCommand(DriveSubsystem m_DriveSubsystem, IntakeSubsystem m_IntakeSubsystem, ShooterSubsystem m_ShooterSubsystem, double duration, String alliance) {
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.alliance = alliance;
    this.duration = duration;
    addRequirements(m_DriveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      shootTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_DriveSubsystem.limelightDrive(0, 0, 0, alliance);
      m_IntakeSubsystem.setAutoFeedWheel(Constants.FEEDSPEED);
      m_ShooterSubsystem.feedForwardPIDShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      shootTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shootTimer.get() > duration)
        return true;
    return false;
  }
}
