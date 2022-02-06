// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public WPI_VictorSPX intake = new WPI_VictorSPX(Constants.INTAKE_FEED);
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
