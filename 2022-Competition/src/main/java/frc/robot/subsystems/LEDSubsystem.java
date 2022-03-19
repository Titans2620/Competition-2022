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
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(Constants.LED);
    led.setLength(Constants.LEDLENGTH);
    ledBuffer = new AddressableLEDBuffer(32);
    led.setData(ledBuffer);
    led.start();
  }

  public void setColor(){
    
  }

  @Override
  public void periodic() {
    for(int i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, 255, 0, 0);
    }
    led.setData(ledBuffer);
  }
}
