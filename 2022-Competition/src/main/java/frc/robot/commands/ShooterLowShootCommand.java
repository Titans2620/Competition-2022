// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterLowShootCommand extends CommandBase {
  /** Creates a new ShooterLowShootCommand. */
  ShooterSubsystem m_ShooterSubsystem;
  ColorSensorSubsystem m_ColorSensorSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  Timer timer;

  String alliance;
  String colorState;
  public ShooterLowShootCommand(ShooterSubsystem m_ShooterSubsystem, ColorSensorSubsystem m_ColorSensorSubsystem, IntakeSubsystem m_IntakeSubsystem, String alliance) {
      this.m_ColorSensorSubsystem = m_ColorSensorSubsystem;
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
      SmartDashboard.putNumber("Low Goal Shooter Timer", timer.get());
      m_IntakeSubsystem.setIntakeRoller(Constants.INTAKESPEED);
      if(m_ColorSensorSubsystem.getColorState() == "red"){
        if(colorState != "red"){
          colorState = "red";
          timer.reset();
        }
      }
      else if(m_ColorSensorSubsystem.getColorState() == "blue"){
        if(colorState != "blue"){
          colorState = "blue";
          timer.reset();
        }
      }

      if(colorState == alliance){
        m_ShooterSubsystem.setShooterLow(.5);
        if(timer.get() > .5){
          m_IntakeSubsystem.setFeedWheel(Constants.FEEDSPEED);
        }
        else{ 
          m_IntakeSubsystem.setFeedWheel(0);
        }
      }
      else{
        m_ShooterSubsystem.setShooterLow(.25);
        if(timer.get() > .5){
          m_IntakeSubsystem.setFeedWheel(Constants.FEEDSPEED);
        }
        else{
          m_IntakeSubsystem.setFeedWheel(0);
        }
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
