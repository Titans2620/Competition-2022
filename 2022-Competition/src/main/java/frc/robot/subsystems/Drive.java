// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    public final double ROBOT_LENGTH = 2;
    public final double ROBOT_WIDTH = 2;


    public Drive() {}

    @Override
    public void periodic() {
      
    }

    public void swerve(double x1, double y1, double x2) {
        y1 *= -1;
        double radius = Math.sqrt(Math.pow(ROBOT_LENGTH, 2) + Math.pow(ROBOT_WIDTH, 2));

        double a = x1 - x2 * (L/r);
    }

}
