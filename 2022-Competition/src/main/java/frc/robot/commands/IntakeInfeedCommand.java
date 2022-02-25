// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInfeedCommand extends CommandBase {
  /*************************************************
   This command will be called when trying to infeed balls. It will consist of the following actions
   1. Intake Roller will be running in.
   2. Intake Arm will be in the down position (If not, move to down position based on limit switch)
   3. Feed Wheel will be running unless a ball it in position based on an undetermined sensor.
  **************************************************/
  public IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  ColorSensorSubsystem m_ColorSensorSubsystem;

  public IntakeInfeedCommand(IntakeSubsystem m_IntakeSubsystem, ColorSensorSubsystem m_ColorSensorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.m_ColorSensorSubsystem = m_ColorSensorSubsystem;

    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_IntakeSubsystem.setIntakeRoller(Constants.INTAKESPEED);
    m_IntakeSubsystem.setFeedWheel(Constants.FEEDSPEED);
    
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
