// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants;

public class DriveLimelightCommand extends CommandBase {
  /********************************************************
  The limelight drive command will only run when the trigger is pressed. This will overwrite the rotation joystick and instead aim the robot towards the goal.
  ***********************************************************/
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_translationZSupplier;
  private double m_rotation;
  public String allianceColor;
  public String lastStateWhenNotFound = "FASTLEFT";

  public DriveLimelightCommand(DriveSubsystem drivetrainSubsystem, DoubleSupplier d, DoubleSupplier e, String allianceColor, DoubleSupplier z) {
      this.m_driveSubsystem = drivetrainSubsystem;
      this.m_translationXSupplier = d;
      this.m_translationYSupplier = e;
      this.m_translationZSupplier = z;
      this.m_rotation = 0.0;
      addRequirements(m_driveSubsystem);
      this.allianceColor = allianceColor;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
        m_driveSubsystem.limelightDrive(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble(), m_translationZSupplier.getAsDouble(), allianceColor);

    }

  @Override
  public void end(boolean interrupted) {
        m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}
