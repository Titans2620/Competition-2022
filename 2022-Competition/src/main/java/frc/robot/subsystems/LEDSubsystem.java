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
  int blinkLength, blinkCount, fadeHueValue, fadeIncrementValue, runLength = 1, runIndex = 0, runCountLength = 5, runCountIndex = 0;
  boolean blinkStateOn, fadeIncMode, altStateHigh;
  static String LEDstate;
  static int LEDpriority = 0;
  int altLength = 25, altCount = 0;

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
    for(int index = 0; index < ledBuffer.getLength(); index++){
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

  public void setRun(int hue){

    if(runCountIndex >= runCountLength){
      for(int index = 0; index < ledBuffer.getLength(); index++){
        ledBuffer.setHSV(index, hue, 255, 0);
      }
      if(runLength != 0){
        for(int index = 0; index < runLength; index++){
          if(runIndex - index < 0){
              int overLoopIndex = runIndex - index;
              ledBuffer.setHSV(ledBuffer.getLength() / 2 - overLoopIndex, hue, 255, 255 * ((runLength - index) / runLength));
              ledBuffer.setHSV(ledBuffer.getLength() / 2 - overLoopIndex + 15, hue, 255, 255 * ((runLength - index) / runLength));
          }
          else{
            ledBuffer.setHSV(runIndex - index, hue, 255, 255 * ((runLength - index) / runLength));
            ledBuffer.setHSV(runIndex - index + 15, hue, 255, 255 * ((runLength - index) / runLength));
          }
          
        }
      }
      if(runIndex == ledBuffer.getLength() / 2){
        runIndex = 0;
      }
      else{
        runIndex++;
      }
      runCountIndex = 0;
    }
    else{
      runCountIndex++;
    }
  }

  public void setalternate(int red, int green, int blue){
      if(altCount < altLength){
          altCount++;

      }
      else{
          if(altStateHigh){
            altStateHigh = false;
            for(int index = 0; index < ledBuffer.getLength(); index+=2){
              ledBuffer.setRGB(index, red, green, blue);
            }
            for(int index = 1; index < ledBuffer.getLength(); index+=2){
              ledBuffer.setRGB(index, 0, 0, 0);
            }
          }
          else{
            altStateHigh = true;
            for(int index = 1; index < ledBuffer.getLength(); index+=2){
              ledBuffer.setRGB(index, red, green, blue);
            }
            for(int index = 0; index < ledBuffer.getLength(); index+=2){
              ledBuffer.setRGB(index, 0, 0, 0);
            }
          }
          altCount = 0;
      }

  }

  public static void setState(String state, int priority){

    if(priority > LEDpriority){
        LEDstate = state;
        LEDpriority = priority;
    }
  }

  @Override
  public void periodic() {
    if(LEDpriority == 0){
        if(DriverStation.getAlliance().toString() == "Red"){
          if(timer.get() < 10)
              LEDstate = "FadeRed";
          else if(DriverStation.getMatchTime() < 30.0 && DriverStation.getMatchTime() != -1.0)
              LEDstate = "AltRed";
          else
              LEDstate = "SolidRed";
        }
        else if(DriverStation.getAlliance().toString() == "Blue"){
          if(timer.get() < 10)
              LEDstate = "FadeBlue";
          else if(DriverStation.getMatchTime() < 30.0 && DriverStation.getMatchTime() != -1.0)
              LEDstate = "AltBlue";
          else
              LEDstate = "SolidBlue";
        }
        else
            LEDstate = "SolidWhite";
    }

    switch(LEDstate){
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
      
      case "RunRed":
          setRun(0);
          break;
      case "RunBlue":
          setRun(120);
          break;

      case "AltRed":
          setalternate(255, 0, 0);
          break;
      case "AltBlue":
          setalternate(0, 0, 255);
          break;
      case "AltYellow":
          setalternate(255, 255, 0);
          break;

      default:
          setSolidColor(255, 255, 255);

    }
    led.setData(ledBuffer);
    LEDstate = "None";
    LEDpriority = 0;
  }
}
