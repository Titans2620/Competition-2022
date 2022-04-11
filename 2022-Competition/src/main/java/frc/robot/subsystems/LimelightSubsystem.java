// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    /********************************************************
  This subsystem will manage the limelight. The purpose of this subsystem is to supply other subsystems the Limelight state and manage the LED.

  Limelight (Networktable: "limelight") - Vision processing tool that will be utilized in the shoot and drive subsystems.

         Table Entry Types
         "ledMode" - changes state of led's
         Values:
            0 - Default from pipeline (preset)
            1 - Force off
            2 - Force Blink
            3 - Force on
        "camMode" - Changes Camera mode
        Values:
            0 - Vision Processing
            1 - Driver camera (Increases exposure, operates like normal camera)
        "pipeline" - Sets current pipeline
        Values:
            0 - Default pipeline
            (1-9) - Custom set pipelines
        "snapshot" - takes snapshot while in operation
        Values:
            0 - stop snapshots
            1 - take two(2) snapshots per second
  
  ***********************************************************/

  private final static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private static String state;
  private static double tx;
  private static double ty;
  private static double ta;

  private static double distance;
  
    private static double angleToGoalDegrees;
    private static double angleToGoalRadians;

    private String alliance;
    private ColorSensorSubsystem m_ColorSensorSubsystem;
    private LEDSubsystem m_LedSubsystem;
  
  public LimelightSubsystem(String alliance, ColorSensorSubsystem m_ColorSensorSubsystem, LEDSubsystem m_LedSubsystem) {
        this.alliance = alliance;
        this.m_ColorSensorSubsystem = m_ColorSensorSubsystem;
        this.m_LedSubsystem = m_LedSubsystem;

  }

  @Override
    public void periodic() {
        getLimelightState();
        SmartDashboard.putString("Limelight State: ", state);
        SmartDashboard.putNumber("Goal Distance", getLimelightDistanceFromGoal());

    }

    public double getLimelightDistanceFromGoal(){

        angleToGoalDegrees = Constants.LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        distance = (Constants.LIMELIGHT_TARGET_HEIGHT_INCHES - Constants.LIMELIGHT_LENSE_FLOOR_HEIGHT)/Math.tan(angleToGoalRadians);

        return distance;
    }
    public String getLimelightState(){
        
        tx = table.getEntry("tx").getDouble(-10000);
        ty = table.getEntry("ty").getDouble(-10000);
        ta = table.getEntry("ta").getDouble(-10000);
        //System.out.println("Alliance: " + alliance + "| color sensor: " + m_ColorSensorSubsystem.getColorState());
        if(m_ColorSensorSubsystem.getColorState() != alliance && m_ColorSensorSubsystem.getColorState() != "neither"){
            //tx += Constants.WRONG_BALL_AIM_VARIANCE_PIXELS;
            /*if(m_ColorSensorSubsystem.getColorState() == "red"){
                m_LedSubsystem.setBlink(255, 0, 0);
            }
            else{
                m_LedSubsystem.setBlink(0, 0, 255);
            }*/
        }

        if(tx == 0.0 && ty == 0.0){
            state = "NOT FOUND";
        }
        else{
            if(m_ColorSensorSubsystem.getColorState() == alliance){
                //m_LedSubsystem.setSolidColor(0, 255, 0);
            }
            if(tx < -4.0 + Constants.LIMELIGHT_X_OFFSET)
                state = "FASTLEFT";
            else if(tx > 4.0  + Constants.LIMELIGHT_X_OFFSET)
                state = "FASTRIGHT";
            else if(tx < - 0.3 + Constants.LIMELIGHT_X_OFFSET && tx > -4.0 + Constants.LIMELIGHT_X_OFFSET)
                state = "SLOWLEFT";
            else if(tx > 0.3 + Constants.LIMELIGHT_X_OFFSET && tx < -4.0 + Constants.LIMELIGHT_X_OFFSET)
                state = "SLOWRIGHT";
            else
                state = Constants.LIMELIGHT_STOP;
            
        }
        return state;
    }

    public void setLimelightLED(String ledState){
        ledState = ledState.toUpperCase();
        switch(ledState){
            case "ON":
                table.getEntry("ledMode").setNumber(3);
                break;
            case "OFF":
                table.getEntry("ledMode").setNumber(1);
                break;
            case "BLINK":
                table.getEntry("ledMode").setNumber(2);
                break;
            case "PIPELINE":
                table.getEntry("ledMode").setNumber(0);
                break;
            default:
                table.getEntry("ledMode").setNumber(1);
                System.out.println("The entered Limelight LED State is incorrect. Valid inputs are ON, OFF, BLINK, and PIPELINE. Unexpected String was " + ledState);
        }
        
    }
    public void setLimelightCamMode(String camState){
        camState = camState.toUpperCase();
        switch(camState){
            case "CAMERA":
                table.getEntry("camMode").setNumber(1);
                break;
            case "SEARCH":
                table.getEntry("camMode").setNumber(0);
                break;
            default:
                table.getEntry("camMode").setNumber(0);
                System.out.println("The entered Limelight Cam State is incorrect. Valid inputs are Camera and Search. Unexpected String was " + camState);
        }
        
    }
}