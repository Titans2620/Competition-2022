// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooter;
  private RelativeEncoder encoder;
  
  NetworkTableEntry isRedAlliance;

  private SparkMaxPIDController shooterPIDController;
  private LimelightSubsystem m_limelightSubsystem;

  double speedBoost;

   // PID coefficients
   double kP = 0.000100; //0.001000 
   double kI = -0.000002;
   double kD = 0.000001; 
   double kIz = 0; 
   double kFF = 0.000195; 
   double kMaxOutput = 1; 
   double kMinOutput = -1;

   double rpmSetPoint;
   double percentOfMaxRPM;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(LimelightSubsystem m_limeLightSubsystem) {
    this.m_limelightSubsystem = m_limeLightSubsystem;
    shooter = new CANSparkMax(Constants.SHOOTER_WHEEL, MotorType.kBrushless);
    shooter.restoreFactoryDefaults();
    encoder = shooter.getEncoder();
    
    shooterPIDController = shooter.getPIDController();
  
    // set PID coefficients
    shooterPIDController.setP(kP);
    shooterPIDController.setI(kI);
    shooterPIDController.setD(kD);
    shooterPIDController.setIZone(kIz);
    shooterPIDController.setFF(kFF);
    shooterPIDController.setOutputRange(kMinOutput, kMaxOutput);
  
    // display PID coefficients on SmartDashboard
    //SmartDashboard.putNumber("P Gain", kP);
    //SmartDashboard.putNumber("I Gain", kI);
    //SmartDashboard.putNumber("D Gain", kD);
    //SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    //SmartDashboard.putNumber("Max Output", kMaxOutput);
    //SmartDashboard.putNumber("Min Output", kMinOutput);

  }

  public void setShooterLow(double speed){
      shooter.set(speed);
  }

  public void setShooter(double speed){
    m_limelightSubsystem.setLimelightCamMode("Search");
    m_limelightSubsystem.setLimelightLED("On");
    shooter.set(speed);
  }

  public void stopShooter(){
    shooter.set(0);
    m_limelightSubsystem.setLimelightCamMode("Camera");
    m_limelightSubsystem.setLimelightLED("Off");
  }

  public void feedForwardPIDShooter(){
    m_limelightSubsystem.setLimelightCamMode("Search");
    m_limelightSubsystem.setLimelightLED("On");

    speedBoost = (m_limelightSubsystem.getLimelightDistanceFromGoal() - Constants.SHOOTER_MIN_DISTANCE_INCHES) / (Constants.SHOOTER_MAX_DISTANCE_INCHES - Constants.SHOOTER_MIN_DISTANCE_INCHES);
    percentOfMaxRPM = Constants.SHOOTER_MIN_SPEED_PERCENT + (speedBoost * (Constants.SHOOTER_MAX_SPEED_PERCENT - Constants.SHOOTER_MIN_SPEED_PERCENT));

    // read PID coefficients from SmartDashboard
    //double p = SmartDashboard.getNumber("P Gain", 0);
    //double i = SmartDashboard.getNumber("I Gain", 0);
    //double d = SmartDashboard.getNumber("D Gain", 0);
    //double iz = SmartDashboard.getNumber("I Zone", 0);
    //double ff = SmartDashboard.getNumber("Feed Forward", 0);
    //double max = SmartDashboard.getNumber("Max Output", 0);
    //double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    /*
    if((p != kP)) { shooterPIDController.setP(p); kP = p; }
    if((i != kI)) { shooterPIDController.setI(i); kI = i; }
    if((d != kD)) { shooterPIDController.setD(d); kD = d; }
    if((iz != kIz)) { shooterPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { shooterPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      shooterPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    */
        rpmSetPoint = percentOfMaxRPM * Constants.SHOOTER_MAX_RPM;
        shooterPIDController.setReference(rpmSetPoint, CANSparkMax.ControlType.kVelocity);
    
    SmartDashboard.putNumber("Target RPM", rpmSetPoint);
  }

  public double getEncoderValue(){
      return encoder.getVelocity();
  }

  public double getTargetRPM(){
    return rpmSetPoint;
  }

  public LimelightSubsystem getLimelight(){
    return m_limelightSubsystem;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Shooter Encoder", encoder.getVelocity());
    //SmartDashboard.putNumber("Variance", encoder.getVelocity() - rpmSetPoint);
  }
}
