// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /********************************************************
  The intake subsystem consists of three motors, a limit switch, and a TBD sensor. When the intake button is pressed, The intake arm will ensure it is always down based on the limit switch. 
  The Intake roller will always be on and Feeder wheel will run until a ball is in position to be shot.

  Intake Roller (CANID: 13) - Rolling cylander on front of intake arm that will pull balls into the robot.
  Intake Rotate (CANID: 14) - Rotating Intake arm that the Intake Roller is attached to.
  Feeder Wheel (CANID: 15) - Lower Internal wheel that will feed balls the the shooter wheel.
  
  Limit Switch (DIO: 0) - Limit switch situated so that it will return true when the intake rotate arm is in position.
  TBD Sensor (?) - Sensor that will detect when the ball is in the correct position to be shot.
  ***********************************************************/
  public WPI_VictorSPX intake = new WPI_VictorSPX(Constants.INTAKE_ROLLER);
  public WPI_VictorSPX intakeRotate = new WPI_VictorSPX(Constants.INTAKE_ROTATE);
  
  public IntakeSubsystem() {}

  public void intake(boolean intakeOn, boolean intakeRotateUp, boolean intakeRotateDown){
    intakeIn(intakeOn);
    intakeRotate(intakeRotateUp, intakeRotateDown);
  }

  public void intakeIn(boolean intakeOn){
    if(intakeOn){
      intake.set(Constants.INTAKESPEED);
    }
    else{
      intake.set(0);
    }
  }

  public void intakeRotate(boolean intakeRotateUp, boolean intakeRotateDown){
    if(intakeRotateUp){
      intakeRotate.set(Constants.INTAKEROTATESPEED);
    }
    else if(intakeRotateDown){
      intakeRotate.set(-Constants.INTAKESPEED);
    }
    else{
      intakeRotate.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
