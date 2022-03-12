// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(Constants.LED);
    ledBuffer = new AddressableLEDBuffer(1);
    led.setData(ledBuffer);
    led.setLength(Constants.LEDLENGTH);
    setColor(Color.kRed);
  }

  public void setColor(Color color){
    ledBuffer.setLED(1, color);
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
