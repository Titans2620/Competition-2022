// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousShootUntilCountCommand extends CommandBase {
  /** Creates a new AutonomousShootUntilCountCommand. */
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  DriveSubsystem m_DriveSubsystem;
  
  int goalCount, currentCount;
  double duration;
  boolean ballClear;

  Timer shooterTimeout = new Timer();

  String alliance;

  public AutonomousShootUntilCountCommand(DriveSubsystem m_DriveSubsystem, IntakeSubsystem m_IntakeSubsystem, ShooterSubsystem m_ShooterSubsystem, int goalCount, double duration, String alliance) {
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.goalCount = goalCount;
    this.duration = duration;
    this.alliance = alliance;
    addRequirements(m_IntakeSubsystem, m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      shooterTimeout.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.setAutoFeedWheelShoot(Constants.FEEDSPEED);
    m_ShooterSubsystem.feedForwardPIDShooter();
    m_DriveSubsystem.limelightDrive(0, 0, 0, alliance);
    if(m_IntakeSubsystem.getColorSensorState() != "neither"){
      if(ballClear){
        currentCount++;
      }
        ballClear = false;
    }
    else{
        ballClear = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      shooterTimeout.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if(shooterTimeout.get() > duration || currentCount == goalCount)
          return true;
    return false;
  }
}
