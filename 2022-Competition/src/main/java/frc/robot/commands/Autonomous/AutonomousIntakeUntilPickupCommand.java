// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonomousIntakeUntilPickupCommand extends CommandBase {
  /** Creates a new AutonomousIntakeUntilPickupCommand. */
  IntakeSubsystem m_IntakeSubsystem;
  ColorSensorSubsystem m_ColorSensorSubsystem;

  int currentCount, goalCount;
  double duration;
  Timer AutoIntakeTimeout = new Timer();
  Boolean ballClear = true;

  public AutonomousIntakeUntilPickupCommand(IntakeSubsystem m_IntakeSubsystem, int goalCount, double duration) {
      this.m_IntakeSubsystem = m_IntakeSubsystem;
      this.goalCount = goalCount;
      this.duration = duration;
      addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AutoIntakeTimeout.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      m_IntakeSubsystem.setAutoFeedWheel(Constants.FEEDSPEED);
      if(m_IntakeSubsystem.isLineSensorObstructed()){
        if(ballClear){
          currentCount++;
        }
          ballClear = false;
      }
      else{
          ballClear = true;
      }
     
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    AutoIntakeTimeout.stop();
    m_IntakeSubsystem.turnOffMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(AutoIntakeTimeout.get() > duration || goalCount == currentCount)
        return true;
    return false;
  }
}
