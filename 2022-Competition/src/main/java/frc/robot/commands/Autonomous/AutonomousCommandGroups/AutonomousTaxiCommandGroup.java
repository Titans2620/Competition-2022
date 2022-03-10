// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.AutonomousCommandGroups;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousTaxiCommandGroup extends SequentialCommandGroup {

DriveSubsystem m_driveSubsystem;
  /** Creates a new AutonomousTaxiCommandGroup. */
  public AutonomousTaxiCommandGroup(DriveSubsystem m_driveSubsystem) {
    this.m_driveSubsystem = m_driveSubsystem;
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(m_driveSubsystem.m_kinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(Math.toRadians(0))),
       List.of(
          new Translation2d(-0.75, 0),
          new Translation2d(-0.75, 0.75),
          new Translation2d(-1.5, 0.75)),
      new Pose2d(2, 0.75, Rotation2d.fromDegrees(Math.toRadians(0))), trajectoryConfig);

      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

      ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, this.m_driveSubsystem::getPose, this.m_driveSubsystem.m_kinematics, xController, yController, thetaController, this.m_driveSubsystem::setModuleStates, this.m_driveSubsystem);
        

    addCommands(
        new InstantCommand(() -> this.m_driveSubsystem.resetOdometry(trajectory.getInitialPose())), 
        new InstantCommand(() -> this.m_driveSubsystem.zeroGyroscope()),
        swerveControllerCommand,
        new InstantCommand(() -> this.m_driveSubsystem.stopModules()));
    } 
}
