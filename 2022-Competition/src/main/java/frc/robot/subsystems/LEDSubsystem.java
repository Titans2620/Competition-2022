// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
public class LEDSubsystem extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  Timer timer;
  int blinkLength, blinkCount;
  boolean blinkStateOn;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(Constants.LED);
    ledBuffer = new AddressableLEDBuffer(32);
    led.setData(ledBuffer);
    led.setLength(Constants.LEDLENGTH);
    timer.start();
    led.start();
    blinkLength = 3;
    blinkCount = 0;
    blinkStateOn = false;
  }

  public void setSolidColor(int red, int green, int blue){
    for(int index = 0; index < 32; index++){
      ledBuffer.setRGB(index, red, green, blue);
    }
  }

  public void setBlink(int red, int green, int blue){
    if(blinkCount > blinkLength){
      if(blinkStateOn){
        blinkStateOn = false;
      }
      else{
        blinkStateOn = true;
      }
      blinkCount = 0;
    }
    if(blinkStateOn){
      for(int index = 0; index < 32; index++){
        ledBuffer.setRGB(index, red, green, blue);
      }
    }
    else{
      for(int index = 0; index < 32; index++){
        ledBuffer.setRGB(index, 0, 0, 0);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
