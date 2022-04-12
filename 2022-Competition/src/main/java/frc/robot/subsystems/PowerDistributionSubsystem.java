// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionSubsystem extends SubsystemBase {
  /** Creates a new PowerDistributionSubsystem. */

  private final PowerDistribution pdp;

  private double maxFrontLeftDrive = 0, maxFrontLeftSteer = 0, maxFrontRightDrive = 0, maxFrontRightSteer = 0, maxBackLeftDrive = 0, maxBackLeftSteer = 0, maxBackRightDrive = 0, maxBackRightSteer = 0;
  
  private static final int FRONT_LEFT_DRIVE_PDP_PORT = 0;
  private static final int FRONT_LEFT_STEER_PDP_PORT = 0;
  private static final int FRONT_RIGHT_DRIVE_PDP_PORT = 0;
  private static final int FRONT_RIGHT_STEER_PDP_PORT = 0;
  private static final int BACK_LEFT_DRIVE_PDP_PORT = 0;
  private static final int BACK_LEFT_STEER_PDP_PORT = 0;
  private static final int BACK_RIGHT_DRIVE_PDP_PORT = 0;
  private static final int BACK_RIGHT_STEER_PDP_PORT = 0;

  public PowerDistributionSubsystem() {
      pdp = new PowerDistribution(0, ModuleType.kCTRE);
  }

  @Override
  public void periodic() {

      if(pdp.getCurrent(FRONT_LEFT_DRIVE_PDP_PORT) > maxFrontLeftDrive)
          maxFrontLeftDrive = pdp.getCurrent(FRONT_LEFT_DRIVE_PDP_PORT);

      if(pdp.getCurrent(FRONT_LEFT_STEER_PDP_PORT) > maxFrontLeftSteer)
          maxFrontLeftSteer = pdp.getCurrent(FRONT_LEFT_STEER_PDP_PORT);

      if(pdp.getCurrent(FRONT_RIGHT_DRIVE_PDP_PORT) > maxFrontRightDrive)
          maxFrontRightDrive = pdp.getCurrent(FRONT_RIGHT_DRIVE_PDP_PORT);

      if(pdp.getCurrent(FRONT_RIGHT_STEER_PDP_PORT) > maxFrontRightSteer)
          maxFrontRightSteer = pdp.getCurrent(FRONT_RIGHT_STEER_PDP_PORT);
          
      if(pdp.getCurrent(BACK_LEFT_DRIVE_PDP_PORT) > maxBackLeftDrive)
          maxBackLeftDrive = pdp.getCurrent(BACK_LEFT_DRIVE_PDP_PORT);

      if(pdp.getCurrent(BACK_LEFT_STEER_PDP_PORT) > maxBackLeftSteer)
          maxBackLeftSteer = pdp.getCurrent(BACK_LEFT_STEER_PDP_PORT);

      if(pdp.getCurrent(BACK_RIGHT_DRIVE_PDP_PORT) > maxBackRightDrive)
          maxBackRightDrive = pdp.getCurrent(BACK_RIGHT_DRIVE_PDP_PORT);

      if(pdp.getCurrent(BACK_RIGHT_STEER_PDP_PORT) > maxBackRightSteer)
          maxBackRightSteer = pdp.getCurrent(BACK_RIGHT_STEER_PDP_PORT);
   
      Shuffleboard.getTab("Current Draws").add("Front Left Drive", pdp.getCurrent(FRONT_LEFT_DRIVE_PDP_PORT));
      Shuffleboard.getTab("Current Draws").add("Front Left Drive Max", maxFrontLeftDrive);
      Shuffleboard.getTab("Current Draws").add("Front Left Steer", pdp.getCurrent(FRONT_LEFT_STEER_PDP_PORT));
      Shuffleboard.getTab("Current Draws").add("Front Left Steer Max", maxFrontLeftSteer);
      Shuffleboard.getTab("Current Draws").add("Front Right Drive", pdp.getCurrent(FRONT_RIGHT_DRIVE_PDP_PORT));
      Shuffleboard.getTab("Current Draws").add("Front Right Drive Max", maxFrontRightDrive);
      Shuffleboard.getTab("Current Draws").add("Front Right Steer", pdp.getCurrent(FRONT_RIGHT_STEER_PDP_PORT));
      Shuffleboard.getTab("Current Draws").add("Front Right Steer Max", maxFrontRightSteer);
      Shuffleboard.getTab("Current Draws").add("Back Left Drive", pdp.getCurrent(BACK_LEFT_DRIVE_PDP_PORT));
      Shuffleboard.getTab("Current Draws").add("Back Left Drive Max", maxBackLeftDrive);
      Shuffleboard.getTab("Current Draws").add("Back Left Steer", pdp.getCurrent(BACK_LEFT_STEER_PDP_PORT));
      Shuffleboard.getTab("Current Draws").add("Back Left Steer Max", maxBackLeftSteer);
      Shuffleboard.getTab("Current Draws").add("Back Right Drive", pdp.getCurrent(BACK_RIGHT_DRIVE_PDP_PORT));
      Shuffleboard.getTab("Current Draws").add("Back Right Drive Max", maxBackRightDrive);
      Shuffleboard.getTab("Current Draws").add("Back Right Steer", pdp.getCurrent(BACK_RIGHT_STEER_PDP_PORT));
      Shuffleboard.getTab("Current Draws").add("Back Right Steer Max", maxBackRightSteer);
  }
}
