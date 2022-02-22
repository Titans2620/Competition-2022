// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /********************************************************
  The shooter subsystem will consist of a shooter motor and will implement the limelight subsystem.

  Shooter Wheel (CANID: 16) - Shooter wheels that will launch the ball towards the goal. These will need to get up to speed before the feeder wheel provides the ball.
  ***********************************************************/
  
  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
