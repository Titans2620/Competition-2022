// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterShootCommand extends CommandBase {
  /***************************************************
   This command will be run when the shoot button is pressed. Both this and the Limelight Drive command will run simultaneously.
   
   This command will turn on the shooter motor and once the motor is at the desired speed and the limelight is reading that we are aimed correctly the feed wheel will turn on.

   1. Turn on Shooter Motor
   2. Check limelight state
   3. Check Motor Speed
      4a. (Limelight state is good & shooter motor is at correct speed) - Turn on Feed Wheel
      4b. (Otherwise) - Leave Feed Wheel off.
   ***************************************************/
  ShooterSubsystem m_ShooterSubsystem;
  LimelightSubsystem m_limeLightSubsystem;

  String alliance;

  public ShooterShootCommand(ShooterSubsystem m_ShooterSubsystem, LimelightSubsystem m_LimelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.m_limeLightSubsystem = m_LimelightSubsystem;

    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    m_limeLightSubsystem.setLimelightCamMode("Search");
    m_limeLightSubsystem.setLimelightLED("On");
    m_ShooterSubsystem.feedForwardPIDShooter(Constants.SHOOTERSPEED);
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
