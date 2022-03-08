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
  private final LimelightSubsystem limelightSubsystem;
  private final ColorSensorSubsystem m_ColorSensorSubsystem;
  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_translationZSupplier;
  private double m_rotation;
  public String allianceColor;
  public String lastStateWhenNotFound = "FASTLEFT";

  public DriveLimelightCommand(DriveSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem, DoubleSupplier d, DoubleSupplier e, ColorSensorSubsystem m_ColorSensorSubsystem, String allianceColor, DoubleSupplier z) {
      this.m_driveSubsystem = drivetrainSubsystem;
      this.m_translationXSupplier = d;
      this.m_translationYSupplier = e;
      this.m_translationZSupplier = z;
      this.m_rotation = 0.0;
      this.limelightSubsystem = limelightSubsystem;
      this.m_ColorSensorSubsystem = m_ColorSensorSubsystem;
      addRequirements(drivetrainSubsystem);
      this.allianceColor = allianceColor;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
      String tableState = limelightSubsystem.getLimelightState();
      if(m_ColorSensorSubsystem.getColorState() == allianceColor){
        switch(tableState){
            case "NOT FOUND":
                //If the limelight does not find the reflective tape we will rotate to attempt to find it. 
                //Note: This will result in a spinning motion if there is an issue with the Limelight detecting the reflective tape.
                //m_rotation = m_translationZSupplier.getAsDouble();
                if(lastStateWhenNotFound == "FASTRIGHT"){
                    m_rotation = Constants.LIMELIGHT_SEARCH_SPEED;
                }
                else{
                    m_rotation = -Constants.LIMELIGHT_SEARCH_SPEED;
                }
                m_rotation = m_translationZSupplier.getAsDouble();
                break;
            case "FASTLEFT":
                //The reflective tape is too far to the left so a CCW (Counterclockwise) rotation will be needed to center the shooter.
                m_rotation = Constants.LIMELIGHT_FAST_SPEED;
                lastStateWhenNotFound = "FASTLEFT";
                break;
            case "SLOWLEFT":
                m_rotation = Constants.LIMELIGHT_SLOW_SPEED;
                break;
            case "FASTRIGHT":
                //The reflective tape is too far to the right so a CCW (Clockwise) rotation will be needed to center the shooter.
                m_rotation = -Constants.LIMELIGHT_FAST_SPEED;
                lastStateWhenNotFound = "FASTRIGHT";
                break;
            case "SLOWRIGHT":
                m_rotation = -Constants.LIMELIGHT_SLOW_SPEED;
                break;
            case "STOP":
                m_rotation = 0.0;
                break;
            default:
                m_rotation = 0.0;
                System.out.println("The Limelight State is an invalid value, Valid states are: NOT FOUND, FASTLEFT, FASTRIGHT, SLOWLEFT, SLOWRIGHT, and STOP. The current state is: " + tableState);
        }
    }
    else{
        m_rotation = m_translationZSupplier.getAsDouble();
    }
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        -m_rotation * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        m_driveSubsystem.getGyroscopeRotation()
                )
        );
      

        SmartDashboard.putNumber("TranslationX", m_translationXSupplier.getAsDouble());
        SmartDashboard.putNumber("TranslationY", m_translationYSupplier.getAsDouble());
        SmartDashboard.putNumber("TranslationR", m_rotation);
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
