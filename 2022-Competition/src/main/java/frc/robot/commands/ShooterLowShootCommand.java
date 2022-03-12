// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterLowShootCommand extends CommandBase {
  /** Creates a new ShooterLowShootCommand. */
  ShooterSubsystem m_ShooterSubsystem;
  ColorSensorSubsystem m_ColorSensorSubsystem;
  Timer timer;

  String alliance;
  public ShooterLowShootCommand(ShooterSubsystem m_ShooterSubsystem, ColorSensorSubsystem m_ColorSensorSubsystem, String alliance) {
      this.m_ColorSensorSubsystem = m_ColorSensorSubsystem;
      this.m_ShooterSubsystem = m_ShooterSubsystem;
      addRequirements(m_ShooterSubsystem);
      timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(m_ColorSensorSubsystem.getColorState() == alliance){
        if(timer.get() < 2){
          m_ShooterSubsystem.setShooterLow(Constants.LOWGOALSHOOTERSPEEDCORRECT);
        }
      }
      else{
          m_ShooterSubsystem.setShooterLow(Constants.LOWGOALSHOOTERSPEEDCORRECT);
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
