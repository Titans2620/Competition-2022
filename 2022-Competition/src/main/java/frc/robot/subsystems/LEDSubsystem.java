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
  String state;
  int priority = 0;
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
    fadeIncrementValue = 5;
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

  public void setState(String state, int priority){

    if(priority > this.priority){
        this.state = state;
        this.priority = priority;
    }
  }

  @Override
  public void periodic() {
    if(priority == 0){
        if(DriverStation.getAlliance().toString() == "Red"){
          if(timer.get() < 10)
              state = "FadeRed";
          else
              state = "SolidRed";
        }
        if(DriverStation.getAlliance().toString() == "Blue"){
          if(timer.get() < 10)
              state = "FadeBlue";
          else
              state = "SolidBlue";
        }
        else
            state = "SolidWhite";
    }

    switch(state){
      case "SolidRed":
          setSolidColor(255, 0, 0);
          break;
      case "SolidBlue":
          setSolidColor(0, 0, 255);
          break;
      case "SolidGreen":
          setSolidColor(0, 255, 0);
          break;
      case "SolidYellow":
          setSolidColor(255, 255, 0);
          break;
      case "SolidWhite":
          setSolidColor(255, 255, 255);
          break;

      case "FadeRed":
          setFade(0);
          break;
      case "FadeBlue":
          setFade(120);
          break;

      case "BlinkRed":
          setBlink(255, 0, 0);
          break;

      case "Blinkblue":
          setBlink(0, 0, 255);
          break;

      default:
          setSolidColor(255, 255, 255);

    }
    led.setData(ledBuffer);
    state = "None";
    priority = 0;
  }
}
