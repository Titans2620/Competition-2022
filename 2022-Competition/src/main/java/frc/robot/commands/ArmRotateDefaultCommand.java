// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmRotateDefaultCommand extends CommandBase {
  /** Creates a new ArmRotateDefaultCommand. */
  ArmSubsystem m_ArmSubsystem;
  IntakeSubsystem m_IntakeSubsystem;
  Timer timer = new Timer();
  
  public ArmRotateDefaultCommand(ArmSubsystem m_ArmSubsystem, IntakeSubsystem m_IntakeSubsystem){
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    addRequirements(m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_IntakeSubsystem.isLineSensorObstructed()){
      timer.reset();
    }
    if(timer.get() > 3.0){
      m_ArmSubsystem.rotateArm(Constants.INTAKEROTATEUPSPEED);
    }
    else{
      m_ArmSubsystem.stopMotor();
    }
    SmartDashboard.putNumber("Arm Rotate Timer", timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
