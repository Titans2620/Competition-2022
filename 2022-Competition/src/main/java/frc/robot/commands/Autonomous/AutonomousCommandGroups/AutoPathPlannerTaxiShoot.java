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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Autonomous.AutonomousShootUntilTimeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPathPlannerTaxiShoot extends SequentialCommandGroup {
  DriveSubsystem m_driveSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  String alliance;

  public AutoPathPlannerTaxiShoot(DriveSubsystem m_driveSubsystem, ShooterSubsystem m_ShooterSubsystem, IntakeSubsystem m_IntakeSubsystem, String alliance) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.alliance = alliance;
    addRequirements(m_driveSubsystem, m_ShooterSubsystem, m_IntakeSubsystem);

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    
    PathPlannerTrajectory taxiShoot = PathPlanner.loadPath("Taxi Shoot", 1, 1);

    PPSwerveControllerCommand taxiShootCommand = new PPSwerveControllerCommand(
      taxiShoot,
        this.m_driveSubsystem::getPose, 
        this.m_driveSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        this.m_driveSubsystem::setModuleStates, 
        this.m_driveSubsystem
    ); 

    addCommands(
      new InstantCommand(() -> this.m_driveSubsystem.setStartingPose(10.36, 4.4, -156.5)),
      taxiShootCommand,
      new AutonomousShootUntilTimeCommand(m_driveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, 10, alliance)
    );
  }
}
