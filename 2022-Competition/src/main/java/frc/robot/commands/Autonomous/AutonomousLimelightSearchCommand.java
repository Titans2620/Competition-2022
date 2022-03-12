// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class AutonomousLimelightSearchCommand extends CommandBase {
  LimelightSubsystem m_limelightSubsystem;
  Timer limelightTimer = new Timer();
  Double duration;

  /** Creates a new LimelightSearchCommand. */
  public AutonomousLimelightSearchCommand(LimelightSubsystem limelightSubsystem, double duration) {
    m_limelightSubsystem = limelightSubsystem;
    this.duration = duration;

    addRequirements(limelightSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelightTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_limelightSubsystem.setLimelightLED("on");
      m_limelightSubsystem.setLimelightCamMode("Search");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      limelightTimer.stop();
      m_limelightSubsystem.setLimelightLED("off");
      m_limelightSubsystem.setLimelightCamMode("camera");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if(limelightTimer.get() > duration){
          return true;
      }
    return false;
  }
}