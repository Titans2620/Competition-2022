// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbPivotSubsystem extends SubsystemBase {
  /** Creates a new ClimbPivotSubsystem. */
  
  PWMTalonSRX climbPivot = new PWMTalonSRX(Constants.PWM_CLIMB_PIVOT);
  
  public ClimbPivotSubsystem() {

  }

  public void pivotClimb(double speed){
      climbPivot.set(speed);
  }

  public void stopMotor(){
      climbPivot.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
