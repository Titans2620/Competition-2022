// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PicoColorSensor;
import frc.robot.PicoColorSensor.RawColor;

public class ColorSensorSubsystem extends SubsystemBase {
  //private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  int red;
  int blue;
  int green;
  String colorState;
  PicoColorSensor picoColor = new PicoColorSensor();
  RawColor picoRawColor;
  /** Creates a new ColorSensor. */
  public ColorSensorSubsystem() {
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    picoRawColor = picoColor.getRawColor0();
      updateColor();
      
      SmartDashboard.putNumber("Red", red);
      SmartDashboard.putNumber("Blue", blue);
      SmartDashboard.putNumber("Green", green);
      SmartDashboard.putString("Color Sensor: ", colorState);

  }

  public void updateColor(){
    
    red = picoRawColor.red;
    blue = picoRawColor.blue;
    green = picoRawColor.green;
    //red = m_colorSensor.getRed();
    //blue = m_colorSensor.getBlue();
    //green = m_colorSensor.getGreen();

    if(red > blue && green > 300 ){
      colorState = "red";
    }
    else if(red < blue && green > 300){
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
