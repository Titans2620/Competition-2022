// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooter;
  private RelativeEncoder encoder;
  
  NetworkTableEntry isRedAlliance;

  public void setShooter(double speed){

    shooter.set(speed);
  }

  public void stopShooter(){
    shooter.set(0);
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooter = new CANSparkMax(Constants.SHOOTER_WHEEL, MotorType.kBrushless);
    encoder = shooter.getEncoder();

  }

  public double getEncoderValue(){
      return encoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Encoder", encoder.getVelocity());
  }
}
