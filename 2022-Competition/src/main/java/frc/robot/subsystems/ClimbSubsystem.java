// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    /********************************************************
  This Command will manage the climb subsystem. The subsystem consists of two motors and two limit switches. There will be an extending motion and retracting motion.
  Both motors will be run at the same time and will run unless a limit switch is hit in both directions.

  Left Climb (CANID: 15) - Climbing motor situated on the left side of the robot.
  Right Climb (CANID: 16) - Climbing motor situated on the right side of the robot.

  Limit Switch (DIO: 1) - Limit switch that will detect when the climb is in the lowest position.
  Limit Switch (DIO: 2) - Limit switch that will detect when the climb is in the highest position.
  
  ***********************************************************/
  public ClimbSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
