// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class LimelightDriveCommand extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private double m_rotation;


  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  public LimelightDriveCommand(DriveSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
      this.m_driveSubsystem = drivetrainSubsystem;
      this.m_translationXSupplier = translationXSupplier;
      this.m_translationYSupplier = translationYSupplier;
      this.m_rotation = 0.0;
      addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    try{
      String tableState = getLimelightState();

      switch(tableState){
          case "NOT FOUND":
              //If the limelight does not find the reflective tape we will rotate to attempt to find it. 
              //Note: This will result in a spinning motion if there is an issue with the Limelight detecting the reflective tape.
              m_rotation = Constants.LIMELIGHT_SEARCH_SPEED;
              break;
          case "FASTLEFT":
              //The reflective tape is too far to the left so a CCW (Counterclockwise) rotation will be needed to center the shooter.
              m_rotation = -Constants.LIMELIGHT_FAST_SPEED;
              break;
          case "SLOWLEFT":
              m_rotation = -Constants.LIMELIGHT_SLOW_SPEED;
              break;
          case "FASTRIGHT":
              //The reflective tape is too far to the right so a CCW (Clockwise) rotation will be needed to center the shooter.
              m_rotation = Constants.LIMELIGHT_FAST_SPEED;
              break;
          case "SLOWRIGHT":
              m_rotation = Constants.LIMELIGHT_SLOW_SPEED;
              break;
          case "STOP":
              m_rotation = 0.0;
              break;
          default:
              m_rotation = 0.0;
              throw new Exception("The Limelight State is an invalid value, Valid states are: NOT FOUND, FASTLEFT, FASTRIGHT, SLOWLEFT, SLOWRIGHT, and STOP. The current state is: " + tableState);
      }

      m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotation,
                        m_driveSubsystem.getGyroscopeRotation()
                )
        );
    } catch(Exception ex){
          System.out.println(ex.toString());

    }

  }

  @Override
  public void end(boolean interrupted) {
      m_driveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  public String getLimelightState(){
      String state = "";

      double tx = table.getEntry("tx").getDouble(-10000);
      double ty = table.getEntry("ty").getDouble(-10000);
      double ta = table.getEntry("ta").getDouble(-10000);

      if(tx == -10000 && ty == -10000){
        state = "Not Found";
      }
      else{

          if(tx < -4)
            state = "fastLeft";
          else if(tx > 4)
            state = "fastRight";
          else if(tx < - 0.75)
            state = "slowLeft";
          else if(tx > 0.75)
            state = "slowRight";
          else
            state = "stop";

            state = state.toUpperCase();
          
      }
      return state;
  }
}