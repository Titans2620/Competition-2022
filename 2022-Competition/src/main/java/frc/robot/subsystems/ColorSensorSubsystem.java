// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensorSubsystem extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  int red;
  int blue;
  int green;
  String colorState;
  /** Creates a new ColorSensor. */
  public ColorSensorSubsystem() {
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      this.red = m_colorSensor.getRed();
      this.blue = m_colorSensor.getBlue();
      this.green = m_colorSensor.getGreen();
      updateColor();
      SmartDashboard.putString("Color", colorState);
  }

  public void updateColor(){
    if(red > 500 && blue < 300){
      colorState = "red";
    }
    else if(red < 500 && blue > 350){
      colorState = "blue";
    }
    else{
      colorState = "neither";
    }
  }
  public String getColorState(){
    return colorState;
  }
}
