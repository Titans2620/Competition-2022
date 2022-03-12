// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    /********************************************************
  This Command will manage the climb subsystem. The subsystem consists of two motors and two limit switches. There will be an extending motion and retracting motion.
  Both motors will be run at the same time and will run unless a limit switch is hit in both directions.
  
  ***********************************************************/

  WPI_VictorSPX leftClimbExtend = new WPI_VictorSPX(Constants.LEFT_CLIMB_EXTEND);
  WPI_VictorSPX rightClimbExtend = new WPI_VictorSPX(Constants.RIGHT_CLIMB_EXTEND);

  PWMTalonSRX leftClimbPivot = new PWMTalonSRX(Constants.PWM_LEFT_CLIMB_PIVOT);
  PWMTalonSRX rightClimbPivot = new PWMTalonSRX(Constants.PWM_LEFT_CLIMB_PIVOT);

  DigitalInput leftClimbLimit = new DigitalInput(Constants.CLIMB_LEFT_LIMIT);
  DigitalInput rightClimbLimit = new DigitalInput(Constants.CLIMB_RIGHT_LIMIT);

  public ClimbSubsystem() {}

  public void doubleClimbExtend(double speed){
    leftClimbExtend.set(speed);
    rightClimbExtend.set(speed);
  }

  public void doubleClimbPivot(double speed){
    leftClimbPivot.set(speed);
    rightClimbPivot.set(speed);
  }

  public void singleClimbExtend(double speed, String side){
    if(side == "left"){
      leftClimbExtend.set(speed);
    }
    else if(side == "right"){
      rightClimbExtend.set(speed);
    }
  }

  public void singleClimbPivot(double speed, String side){
    if(side == "left"){
      leftClimbPivot.set(speed);
    }
    else if(side == "right"){
      rightClimbPivot.set(speed);
    }
  }

  public void stopAll(){
    leftClimbExtend.set(0);
    rightClimbExtend.set(0);
    leftClimbPivot.set(0);
    rightClimbPivot.set(0);
  }
  @Override
  public void periodic() {


  }
}
