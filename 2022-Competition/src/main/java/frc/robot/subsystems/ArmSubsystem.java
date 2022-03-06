// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  WPI_VictorSPX intakeRotate = new WPI_VictorSPX(Constants.INTAKE_ROTATE);
  DigitalInput limit;

  public ArmSubsystem() {
    limit = new DigitalInput(Constants.INTAKE_ROTATE_LIMIT);
  }

  public void rotateArm(double speed){
    intakeRotate.set(speed);
  }

  public void autoRotateArm(Double speed){
    if((!limit.get() && speed < 0) || speed > 0){
      intakeRotate.set(speed);
    }
    else{
      intakeRotate.set(0);
    }
  }

  public void stopMotor(){
    intakeRotate.set(0);
  }
  @Override
  public void periodic() {
  }
}
