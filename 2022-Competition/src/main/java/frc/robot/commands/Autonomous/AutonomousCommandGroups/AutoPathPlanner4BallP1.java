// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.AutonomousCommandGroups;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Autonomous.AutonomousIntakeUntilTimeCommand;
import frc.robot.commands.Autonomous.AutonomousShootUntilTimeCommand;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPathPlanner4BallP1 extends SequentialCommandGroup {
  /** Creates a new AutoPathPlanner4Ball. */
    DriveSubsystem m_driveSubsystem;
    IntakeSubsystem m_IntakeSubsystem;
    ArmSubsystem m_ArmSubsystem;
    ShooterSubsystem m_ShooterSubsystem;
    LimelightSubsystem m_LimelightSubsystem;
    String alliance;
  
    public AutoPathPlanner4BallP1(DriveSubsystem m_driveSubsystem, IntakeSubsystem m_IntakeSubsystem, ArmSubsystem m_ArmSubsystem, ShooterSubsystem m_ShooterSubsystem, LimelightSubsystem m_LimelightSubsystem, String alliance) {
      this.m_driveSubsystem = m_driveSubsystem;
      this.m_IntakeSubsystem = m_IntakeSubsystem;
      this.m_ArmSubsystem = m_ArmSubsystem;
      this.m_ShooterSubsystem = m_ShooterSubsystem;
      this.alliance = alliance;
      addRequirements(m_driveSubsystem, m_IntakeSubsystem, m_ArmSubsystem, m_ShooterSubsystem);
  
      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  
      ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

      PathPlannerTrajectory red4BallS1 = PathPlanner.loadPath("Four Ball P1 Stage 1", 1, 1);
      PathPlannerTrajectory red4BallS2 = PathPlanner.loadPath("Four Ball P1 Stage 2", 1, 1);
      PathPlannerTrajectory red4BallS3 = PathPlanner.loadPath("Four Ball P1 Stage 3", 1, 1);

      PPSwerveControllerCommand red4BallS1Command = new PPSwerveControllerCommand(
        red4BallS1,
        this.m_driveSubsystem::getPose, 
        this.m_driveSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        this.m_driveSubsystem::setModuleStates, 
        this.m_driveSubsystem
    ); 

    PPSwerveControllerCommand red4BallS2Command = new PPSwerveControllerCommand(
        red4BallS2,
        this.m_driveSubsystem::getPose, 
        this.m_driveSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        this.m_driveSubsystem::setModuleStates, 
        this.m_driveSubsystem
    ); 

    PPSwerveControllerCommand red4BallS3Command = new PPSwerveControllerCommand(
        red4BallS3,
        this.m_driveSubsystem::getPose, 
        this.m_driveSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        this.m_driveSubsystem::setModuleStates, 
        this.m_driveSubsystem
    ); 

    addCommands(
      new InstantCommand(() -> this.m_driveSubsystem.setStartingPose(10.37, 3.13, -45)), //Intialize 
      new ParallelCommandGroup(red4BallS1Command, new AutonomousIntakeUntilTimeCommand(m_IntakeSubsystem, m_ArmSubsystem, 3)),
      new AutonomousShootUntilTimeCommand(m_driveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, 2, alliance),
      red4BallS2Command,
      new AutonomousIntakeUntilTimeCommand(m_IntakeSubsystem, m_ArmSubsystem, 4),
      red4BallS3Command,
      new AutonomousShootUntilTimeCommand(m_driveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem,  2, alliance),
      new InstantCommand(() -> this.m_driveSubsystem.stopModules())
    );
  }
}
