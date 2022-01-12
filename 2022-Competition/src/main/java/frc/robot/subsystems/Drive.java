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

        double a = x1 - x2 * (ROBOT_LENGTH / radius);
        double b = x1 + x2 * (ROBOT_LENGTH / radius);
        double c = y1 - x2 * (ROBOT_WIDTH / radius);
        double d = y1 + x2 * (ROBOT_WIDTH / radius);

        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

        double backRightAngle = Math.atan2 (a, d) / Math.PI;
        double backLeftAngle = Math.atan2 (a, c) / Math.PI;
        double frontRightAngle = Math.atan2 (b, d) / Math.PI;
        double frontLeftAngle = Math.atan2 (b, c) / Math.PI;
    }

}
