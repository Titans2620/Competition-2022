// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbPivotSubsystem extends SubsystemBase {
  /** Creates a new ClimbPivotSubsystem. */
  
  PWMTalonSRX climbPivot = new PWMTalonSRX(Constants.PWM_CLIMB_PIVOT);
  DigitalInput leftClimbForwardLimit;
  DigitalInput leftClimbBackLimit;
  DigitalInput rightClimbBackLimit;
  DigitalInput rightClimbForwardLimit;
  
  
  public ClimbPivotSubsystem() {
    leftClimbForwardLimit = new DigitalInput(5);
    leftClimbBackLimit = new DigitalInput(6);
    rightClimbBackLimit = new DigitalInput(7);
    rightClimbForwardLimit = new DigitalInput(8);
    
  }

  public void pivotClimb(double speed){
      climbPivot.set(speed);
  }

  public void autoPivotClimb(double speed){

      if(speed > 0.0 && (leftClimbForwardLimit.get() && rightClimbForwardLimit.get())){
          climbPivot.set(0);
      }
      else if(speed < 0.0 && leftClimbBackLimit.get() && !rightClimbBackLimit.get()){
          climbPivot.set(-.15);
      }
      /*
      else if((speed > 0.0 && (leftClimbForwardLimit.get())) || (speed > 0.0 && (rightClimbForwardLimit.get()))){
          climbPivot.set(.25);
      }
      else if((speed < 0.0 && (leftClimbBackLimit.get())) || (speed > 0.0 && (!rightClimbBackLimit.get()))){
        climbPivot.set(-.25);
      }
      */                                                                                      
      else{
        climbPivot.set(speed);
      }

  }

  public void stopMotor(){
      climbPivot.set(0);
  }

  @Override
  public void periodic() {
    /*
    SmartDashboard.putBoolean("left Forward", leftClimbForwardLimit.get());
    SmartDashboard.putBoolean("left Backward", leftClimbBackLimit.get());
    SmartDashboard.putBoolean("Right Forward", rightClimbForwardLimit.get());
    SmartDashboard.putBoolean("Right Backward", !rightClimbBackLimit.get());
    */

  }
}
