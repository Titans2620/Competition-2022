// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.AutonomousCommandGroups;

import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DriveLimelightCommand;
import frc.robot.commands.LimelightSearchCommand;
import frc.robot.commands.Autonomous.AutonomousIntakeCommand;
import frc.robot.commands.Autonomous.AutonomousIntakeUntilPickupCommand;
import frc.robot.commands.Autonomous.AutonomousLimelightSearchCommand;
import frc.robot.commands.Autonomous.AutonomousShootCommand;
import frc.robot.commands.Autonomous.AutonomousShootUntilCountCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousPathPlannerTestCommandGroup extends SequentialCommandGroup {
  /** Creates a new AutonomousPathPlannerTestCommandGroup. */
  DriveSubsystem m_driveSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  ArmSubsystem m_ArmSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  LimelightSubsystem m_LimelightSubsystem;
  String alliance;

  public AutonomousPathPlannerTestCommandGroup(DriveSubsystem m_driveSubsystem, IntakeSubsystem m_IntakeSubsystem, ArmSubsystem m_ArmSubsystem, ShooterSubsystem m_ShooterSubsystem, LimelightSubsystem m_LimelightSubsystem, String alliance) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.alliance = alliance;
    addRequirements(m_driveSubsystem, m_IntakeSubsystem, m_ArmSubsystem, m_ShooterSubsystem);

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    
    PathPlannerTrajectory red5BallS1 = PathPlanner.loadPath("Red 5 Ball Stage 1", 4, 2);
    PathPlannerTrajectory red5BallS2 = PathPlanner.loadPath("Red 5 Ball Stage 2", 4, 2);
    PathPlannerTrajectory red5BallS3 = PathPlanner.loadPath("Red 5 Ball Stage 3", 4, 2);
    PathPlannerTrajectory red5BallS4 = PathPlanner.loadPath("Red 5 Ball Stage 4", 4, 2);
    PathPlannerTrajectory red5BallS5 = PathPlanner.loadPath("Red 5 Ball Stage 5", 4, 2);

    PPSwerveControllerCommand red5BallS1Command = new PPSwerveControllerCommand(
        red5BallS1,
        this.m_driveSubsystem::getPose, 
        this.m_driveSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        this.m_driveSubsystem::setModuleStates, 
        this.m_driveSubsystem
    );  

    PPSwerveControllerCommand red5BallS2Command = new PPSwerveControllerCommand(
        red5BallS2,
        this.m_driveSubsystem::getPose, 
        this.m_driveSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        this.m_driveSubsystem::setModuleStates, 
        this.m_driveSubsystem
    );

    PPSwerveControllerCommand red5BallS3Command = new PPSwerveControllerCommand(
        red5BallS3,
        this.m_driveSubsystem::getPose, 
        this.m_driveSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        this.m_driveSubsystem::setModuleStates, 
        this.m_driveSubsystem
    );

    PPSwerveControllerCommand red5BallS4Command = new PPSwerveControllerCommand(
        red5BallS4,
        this.m_driveSubsystem::getPose, 
        this.m_driveSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        this.m_driveSubsystem::setModuleStates, 
        this.m_driveSubsystem
    );

    PPSwerveControllerCommand red5BallS5Command = new PPSwerveControllerCommand(
        red5BallS5,
        this.m_driveSubsystem::getPose, 
        this.m_driveSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        this.m_driveSubsystem::setModuleStates, 
        this.m_driveSubsystem
    );

    addCommands(
      new InstantCommand(() -> this.m_driveSubsystem.setStartingPose(8.92, 6.30, 90.00)), //Intialize 
      new ParallelCommandGroup(red5BallS1Command, new AutonomousIntakeCommand(2, m_IntakeSubsystem, m_ArmSubsystem)), //Drive up to first ball with intake on
      new AutonomousIntakeUntilPickupCommand(m_IntakeSubsystem, 1, 2), //Run intake until ball enters
      red5BallS2Command, //Move and turn into shooting position
      new ParallelCommandGroup( //Shoot first two balls
            new AutonomousShootUntilCountCommand(m_driveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, 2, 2, alliance), //Shoot until two balls have shot
            new AutonomousLimelightSearchCommand(m_LimelightSubsystem, 2)
      ),
      new ParallelCommandGroup(red5BallS3Command, new AutonomousIntakeCommand(1, m_IntakeSubsystem, m_ArmSubsystem)), //Grab 3rd ball
      new AutonomousIntakeUntilPickupCommand(m_IntakeSubsystem, 1, 2), //Run intake until ball enters
      new ParallelCommandGroup( //Shoot until ball is gone
        new AutonomousShootUntilCountCommand(m_driveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, 1, 2, alliance), //Shoot until a ball has shot
        new AutonomousLimelightSearchCommand(m_LimelightSubsystem, 2)
      ),
      new ParallelCommandGroup(red5BallS4Command, new AutonomousIntakeCommand(1, m_IntakeSubsystem, m_ArmSubsystem)), //Drive up to driver station with intake on
      new AutonomousIntakeUntilPickupCommand(m_IntakeSubsystem, 2, 2), //Run intake until 2 balls enter
      red5BallS5Command,
      new ParallelCommandGroup( //Shoot until ball is gone
          new AutonomousShootUntilCountCommand(m_driveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, 2, 2, alliance), //Shoot until two balls have shot
          new AutonomousLimelightSearchCommand(m_LimelightSubsystem, 2)
      ),

      new InstantCommand(() -> this.m_driveSubsystem.stopModules())
    );
  }
}
