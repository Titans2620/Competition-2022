// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousTaxiShootCommandGroup extends SequentialCommandGroup {
  DriveSubsystem m_driveSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  /** Creates a new AutonomousTaxiShootCommandGroup. */
  public AutonomousTaxiShootCommandGroup(DriveSubsystem m_driveSubsystem, IntakeSubsystem m_intakeSubsystem, ShooterSubsystem m_ShooterSubsystem) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    addRequirements(m_driveSubsystem, m_intakeSubsystem, m_ShooterSubsystem);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(m_driveSubsystem.m_kinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)),
       List.of(
          new Translation2d(1,0),
          new Translation2d(1,-1)),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig);

      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

      ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, this.m_driveSubsystem::getPose, this.m_driveSubsystem.m_kinematics, xController, yController, thetaController, m_driveSubsystem::setModuleStates, this.m_driveSubsystem);
        
    addCommands(
        new InstantCommand(() -> this.m_driveSubsystem.resetOdometry(trajectory.getInitialPose())), 
        swerveControllerCommand,
        new InstantCommand(() -> this.m_driveSubsystem.stopModules()));
    
  }
}