// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterManualShootCommand extends CommandBase {
  /****************************************
    This command will manually shoot the ball without checking for sensors. 
    
    When the shoot button is pressed, we will still check for motor speed and as long as we are up to speed we will shoot without checking the Limelight
  ******************************************/
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  Timer timer = new Timer();
  public ShooterManualShootCommand(ShooterSubsystem m_ShooterSubsystem, IntakeSubsystem m_IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    addRequirements(m_ShooterSubsystem, m_IntakeSubsystem);
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      LEDSubsystem.setState("SolidYellow", 5);
      m_ShooterSubsystem.setShooter(Constants.SHOOTER_MIN_SPEED_PERCENT);
      if(timer.get() > 3){
          m_IntakeSubsystem.setFeedWheel(Constants.FEEDSPEED);
      }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
