// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final SwerveModule m_frontLeft = new SwerveModule(
      Constants.kFrontLeftDriveMotorPort,
      Constants.kFrontLeftTurningMotorPort,
      Constants.kFrontLeftDriveEncoderPorts,
      Constants.kFrontLeftTurningEncoderPorts,
      Constants.kFrontLeftDriveEncoderReversed,
      Constants.kFrontLeftTurningEncoderReversed);

    private final SwerveModule m_rearLeft = new SwerveModule(
      Constants.kRearLeftDriveMotorPort,
      Constants.kRearLeftTurningMotorPort,
      Constants.kRearLeftDriveEncoderPorts,
      Constants.kRearLeftTurningEncoderPorts,
      Constants.kRearLeftDriveEncoderReversed,
      Constants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight = new SwerveModule(
      Constants.kFrontRightDriveMotorPort,
      Constants.kFrontRightTurningMotorPort,
      Constants.kFrontRightDriveEncoderPorts,
      Constants.kFrontRightTurningEncoderPorts,
      Constants.kFrontRightDriveEncoderReversed,
      Constants.kFrontRightTurningEncoderReversed);

    private final SwerveModule m_rearRight = new SwerveModule(
      Constants.kRearRightDriveMotorPort,
      Constants.kRearRightTurningMotorPort,
      Constants.kRearRightDriveEncoderPorts,
      Constants.kRearRightTurningEncoderPorts,
      Constants.kRearRightDriveEncoderReversed,
      Constants.kRearRightTurningEncoderReversed);

    private final Gyro m_gyro = new ADXRS450_Gyro();
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.kDriveKinematics, m_gyro.getRotation2d());

    public DriveSubsystem() {}

    @Override
    public void periodic() {
        m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_rearLeft.getState(),
        m_frontRight.getState(),
        m_rearRight.getState());
  }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
            //Nick -> This is called a Ternary Operator. Basically it takes the first statement and if true it will run the second statement. Else it will run the third statement.
            //First Statement ? Second Statement : Third Statement

            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kMaxSpeedMetersPerSecond);

      m_frontLeft.setDesiredState(desiredStates[0]);
      m_frontRight.setDesiredState(desiredStates[1]);
      m_rearLeft.setDesiredState(desiredStates[2]);
      m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
      m_frontLeft.resetEncoders();
      m_rearLeft.resetEncoders();
      m_frontRight.resetEncoders();
      m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
      m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
      return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
      return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }
}
