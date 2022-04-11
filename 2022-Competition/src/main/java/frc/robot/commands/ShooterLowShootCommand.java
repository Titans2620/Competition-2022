// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterLowShootCommand extends CommandBase {
  /** Creates a new ShooterLowShootCommand. */
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  Timer timer;

  String alliance;
  public ShooterLowShootCommand(ShooterSubsystem m_ShooterSubsystem, IntakeSubsystem m_IntakeSubsystem) {
      this.m_ShooterSubsystem = m_ShooterSubsystem;
      this.m_IntakeSubsystem = m_IntakeSubsystem;
      addRequirements(m_ShooterSubsystem, m_IntakeSubsystem);
      timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() < 1){
      m_ShooterSubsystem.setShooterLow(Constants.LOWGOALSHOOTERSPEED);
    }
    else{
      m_ShooterSubsystem.setShooter(Constants.LOWGOALSHOOTERSPEED);
        m_IntakeSubsystem.setFeedWheel(Constants.FEEDSPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
