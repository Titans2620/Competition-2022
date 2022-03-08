// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeShootCommand extends CommandBase {
  /** Creates a new IntakeShootCommand. */
  IntakeSubsystem m_IntakeSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  ColorSensorSubsystem m_ColorSensorSubsystem;
  String alliance;
  public IntakeShootCommand(IntakeSubsystem m_IntakeSubsystem, ShooterSubsystem m_ShooterSubsystem, ColorSensorSubsystem m_ColorSensorSubsystem, String alliance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.m_ColorSensorSubsystem = m_ColorSensorSubsystem;
    this.alliance = alliance;
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    double variance = m_ShooterSubsystem.getTargetRPM() - m_ShooterSubsystem.getEncoderValue();
    if((alliance != m_ColorSensorSubsystem.getColorState()) || (m_ShooterSubsystem.getLimelight().getLimelightState() == Constants.LIMELIGHT_STOP && Math.abs(variance) < 100)){

      m_IntakeSubsystem.setFeedWheel(Constants.FEEDSPEED);
    }
    else{
      m_IntakeSubsystem.setFeedWheel(0);
     }
     SmartDashboard.putNumber("variance", variance);
    
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
