// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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
  private WPI_VictorSPX intakeRoller = new WPI_VictorSPX(Constants.INTAKE_ROLLER);
  private WPI_VictorSPX feedWheel = new WPI_VictorSPX(Constants.FEED_WHEEL);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  Color detectedColor;
  double IR;

  //private DigitalInput intakeRotateLimitSwitch = new DigitalInput(Constants.INTAKE_ROTATE_LIMIT);
  
  public IntakeSubsystem() {}

  public void setIntakeRoller(double speed){
      intakeRoller.set(speed);
  }

  public void setFeedWheel(double speed){
      feedWheel.set(speed);
  }

  public void turnOffMotors(){
      intakeRoller.set(0);
      feedWheel.set(0);
  }

  @Override
  public void periodic() {
      detectedColor = m_colorSensor.getColor();
      IR = m_colorSensor.getIR();

      SmartDashboard.putNumber("Red", m_colorSensor.getRed());
      SmartDashboard.putNumber("Green", m_colorSensor.getGreen());
      SmartDashboard.putNumber("Blue", m_colorSensor.getBlue());
      SmartDashboard.putNumber("IR", m_colorSensor.getIR());
  }
}
