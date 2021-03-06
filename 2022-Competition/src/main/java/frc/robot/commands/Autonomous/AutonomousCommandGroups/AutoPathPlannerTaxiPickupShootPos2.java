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
import frc.robot.commands.Autonomous.AutonomousIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.Autonomous.AutonomousShootUntilTimeCommand;
import frc.robot.commands.Autonomous.AutonomousIntakeUntilTimeCommand;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPathPlannerTaxiPickupShootPos2 extends SequentialCommandGroup {
  /** Creates a new AutoPathPlannerTaxiPickupShootPos2. */
    DriveSubsystem m_driveSubsystem;
    IntakeSubsystem m_IntakeSubsystem;
    ArmSubsystem m_ArmSubsystem;
    ShooterSubsystem m_ShooterSubsystem;
    LimelightSubsystem m_LimelightSubsystem;
    String alliance;

  public AutoPathPlannerTaxiPickupShootPos2(DriveSubsystem m_driveSubsystem, IntakeSubsystem m_IntakeSubsystem, ArmSubsystem m_ArmSubsystem, ShooterSubsystem m_ShooterSubsystem, LimelightSubsystem m_LimelightSubsystem, String alliance) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.alliance = alliance;
    addRequirements(m_driveSubsystem, m_IntakeSubsystem, m_ArmSubsystem, m_ShooterSubsystem);

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    
    PathPlannerTrajectory taxiPickupShootS1 = PathPlanner.loadPath("Taxi Pickup Shoot Stage 1 Pos 2", 1, 1);

    PPSwerveControllerCommand taxiPickupShootS1Command = new PPSwerveControllerCommand(
      taxiPickupShootS1,
        this.m_driveSubsystem::getPose, 
        this.m_driveSubsystem.m_kinematics, 
        xController, 
        yController, 
        thetaController, 
        this.m_driveSubsystem::setModuleStates, 
        this.m_driveSubsystem
    ); 

    
    addCommands(
        new InstantCommand(() -> this.m_driveSubsystem.setStartingPose(9.79, 5.55, 29)),
        new ParallelCommandGroup(taxiPickupShootS1Command, new AutonomousIntakeUntilTimeCommand(m_IntakeSubsystem, m_ArmSubsystem, 5)),
        new AutonomousShootUntilTimeCommand(m_driveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, 5, alliance)
    );
  }
}
