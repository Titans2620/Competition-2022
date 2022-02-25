// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
  Timer timer = new Timer();
  ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  LimelightSubsystem m_limeLightSubsystem = new LimelightSubsystem();
  IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  public ShooterShootCommand(ShooterSubsystem m_ShooterSubsystem, IntakeSubsystem m_IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.m_limeLightSubsystem = m_limeLightSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    addRequirements(m_ShooterSubsystem);
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(timer.get() < 3){
      m_ShooterSubsystem.setShooter(Constants.SHOOTERSPEED);
    }
    else{
      m_ShooterSubsystem.setShooter(Constants.SHOOTERSPEED);
      new IntakeShootCommand(m_IntakeSubsystem);
    }
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
