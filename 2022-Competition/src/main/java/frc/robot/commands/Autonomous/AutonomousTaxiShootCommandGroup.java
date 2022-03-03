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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.IntakeInfeedCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousTaxiShootCommandGroup extends CommandBase {
  private DriveSubsystem m_DriveSubsystem;
  private IntakeSubsystem m_IntakeSubsystem;
  private ShooterSubsystem m_ShooterSubsystem;
  private ColorSensorSubsystem m_ColorSensorSubsystem;
  private LimelightSubsystem m_LimelightSubsystem;
  /** Creates a new AutonomousTaxiShoot. */
  public AutonomousTaxiShootCommandGroup(DriveSubsystem m_DriveSubsystem, IntakeSubsystem m_IntakeSubsystem, ShooterSubsystem m_ShooterSubsystem, ColorSensorSubsystem m_ColorSensorSubsystem, LimelightSubsystem m_LimelightSubsystem) {
      this.m_DriveSubsystem = m_DriveSubsystem;
      this.m_IntakeSubsystem = m_IntakeSubsystem;
      this.m_ShooterSubsystem = m_ShooterSubsystem;
      this.m_ColorSensorSubsystem = m_ColorSensorSubsystem;
      this.m_LimelightSubsystem = m_LimelightSubsystem;

      addRequirements(this.m_DriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxSpeedMetersPerSecond).setKinematics(m_DriveSubsystem.m_kinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)),
        List.of(
          new Translation2d(3,0),
          new Translation2d(3, -3)),
          new Pose2d(2, -1, Rotation2d.fromDegrees(0)),
        trajectoryConfig
        );

      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

      ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, m_DriveSubsystem::getPose, m_DriveSubsystem.m_kinematics, xController, yController, thetaController, m_DriveSubsystem::setModuleStates, m_DriveSubsystem);

      IntakeInfeedCommand m_IntakeInfeedCommand = new IntakeInfeedCommand(m_IntakeSubsystem, m_ColorSensorSubsystem);

      ShooterShootCommand m_ShooterShootCommand = new ShooterShootCommand(m_ShooterSubsystem, m_LimelightSubsystem);

        new SequentialCommandGroup(
            new InstantCommand(() -> m_DriveSubsystem.resetOdometry(trajectory.getInitialPose())), 
            new ParallelCommandGroup(swerveControllerCommand, m_IntakeInfeedCommand),
            new InstantCommand(() -> m_DriveSubsystem.stopModules())
            );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
