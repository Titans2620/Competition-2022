// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterManualShootCommand extends CommandBase {
  /****************************************
    This command will manually shoot the ball without checking for sensors. 
    
    When the shoot button is pressed, we will still check for motor speed and as long as we are up to speed we will shoot without checking the Limelight
  ******************************************/
  public ShooterManualShootCommand(ShooterSubsystem m_ShooterSubsystem, LimelightSubsystem m_lLimelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
