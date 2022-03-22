// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
public class LEDSubsystem extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  Timer timer = new Timer();
  int blinkLength, blinkCount, fadeHueValue, fadeIncrementValue;
  boolean blinkStateOn, fadeIncMode;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(Constants.LED);
    ledBuffer = new AddressableLEDBuffer(32);
    led.setLength(Constants.LEDLENGTH);
    led.setData(ledBuffer);
    timer.start();
    led.start();
    blinkLength = 10;
    blinkCount = 0;
    blinkStateOn = false;
    fadeHueValue = 0;
    fadeIncMode = true;
    fadeIncrementValue = 3;
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

  public void setFade(int hue){
      if(fadeIncMode){
          fadeHueValue += fadeIncrementValue;
          if(fadeHueValue > 255){
              fadeHueValue = 255;
              fadeIncMode = false;
          }
      }
      else{
          fadeHueValue -= fadeIncrementValue;
          if(fadeHueValue < 0){
              fadeHueValue = 0;
              fadeIncMode = true;
          }
      }

      for(int index = 0; index < ledBuffer.getLength(); index++){
        ledBuffer.setHSV(index, hue, 255, fadeHueValue);
      }
  }

  public void setDefaultState(String alliance){
    if(timer.get() < 10){
        if(alliance == "Red"){
            setFade(0);
        }
        else{
            setFade(120);
        }
    }
    else{
        if(alliance == "Red"){
            setSolidColor(255, 0, 0);
        }
        else{
            setSolidColor(0, 0, 255);
        }
    }
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
  }
}
