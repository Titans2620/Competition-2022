// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousBasicMovementCommand extends CommandBase {

  private double xSpeed, xSpeedInitial, ySpeed, ySpeedInitial, rotationSpeed, rotationTurnDegreesInitial, rotationVariance, desiredRotationAngle;
  private DriveSubsystem m_DriveSubsystem;
  private Timer movementTimer = new Timer();
  private double remainingTime, movementTimerLength;
  private boolean rotationIsCorrect = false;
  
  /** Creates a new AutonomousBasicMovementCommand. */
  public AutonomousBasicMovementCommand(double xSpeedInitial, double ySpeedInitial, double rotationTurnDegreesInitial, double movementTimerLength, DriveSubsystem m_driveSubsystem) {
    this.xSpeedInitial = xSpeedInitial;
    this.ySpeedInitial = ySpeedInitial;
    this.rotationTurnDegreesInitial = rotationTurnDegreesInitial;
    this.m_DriveSubsystem = m_driveSubsystem;
    this.movementTimerLength = movementTimerLength;
    
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      movementTimer.start();
      desiredRotationAngle = m_DriveSubsystem.getGyroscopeRotation().getDegrees() + rotationTurnDegreesInitial;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      remainingTime = movementTimerLength - movementTimer.get();
      rotationVariance = m_DriveSubsystem.getGyroscopeRotation().getDegrees() - desiredRotationAngle;

      if(remainingTime > 0.25){
          xSpeed = xSpeedInitial;
          ySpeed = ySpeedInitial;
      }
      else{
          xSpeed = xSpeedInitial * 0.1;
          ySpeed = ySpeedInitial * 0.1;
      }

      if(rotationVariance > 25){
          rotationSpeed = Constants.AutoConstants.BASIC_ROTATION_FAST_SPEED;
      }
      else if(rotationVariance < -25){
          rotationSpeed = -Constants.AutoConstants.BASIC_ROTATION_FAST_SPEED;
      }
      else if(rotationVariance > 2){
          rotationSpeed = Constants.AutoConstants.BASIC_ROTATION_FAST_SPEED;
      }
      else if(rotationVariance < -2){
          rotationSpeed = -Constants.AutoConstants.BASIC_ROTATION_FAST_SPEED;
      }
      else{
          rotationIsCorrect = true;
          rotationSpeed = 0.0;
      }
      
      m_DriveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed,
                        ySpeed,
                        rotationSpeed,
                        m_DriveSubsystem.getGyroscopeRotation()
                )
      );

      SmartDashboard.putNumber("Autonomous Timer", movementTimer.get());
      SmartDashboard.putNumber("Rotation Variance", rotationVariance);
      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      movementTimer.stop();
      m_DriveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(remainingTime < 0 && rotationIsCorrect)
        return true;
    return false;
  }
}
