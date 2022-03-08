// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousBasicMovementCommand extends CommandBase {

  private double xVariance, yVariance, rVariance;
  private double xSpeed, ySpeed, xDistance, yDistance;
  private DriveSubsystem m_DriveSubsystem;
  private Rotation2d rotation;
  /** Creates a new AutonomousBasicMovementCommand. */
  public AutonomousBasicMovementCommand(double xSpeed, double ySpeed, double rotationDegrees, double xDistance, double yDistance, DriveSubsystem m_driveSubsystem) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotation = new Rotation2d(rotationDegrees);
    this.xDistance = xDistance;
    this.yDistance = yDistance;
    this.m_DriveSubsystem = m_driveSubsystem;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      xVariance = xDistance - m_DriveSubsystem.getPose().getX();
      yVariance = yDistance - m_DriveSubsystem.getPose().getY();

      m_DriveSubsystem.getPose().getRotation().rotateBy(rotation);

      
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
