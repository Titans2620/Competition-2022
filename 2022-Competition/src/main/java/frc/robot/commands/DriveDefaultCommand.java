package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDefaultCommand extends CommandBase {
    /********************************************************
    The default drive should look for the joystick values for movement in the x and y directions as well as rotation.
    ***********************************************************/
    private final DriveSubsystem m_driveSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private BooleanSupplier fieldOrientation;

    public DriveDefaultCommand(DriveSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier, BooleanSupplier fieldOrientation) {
        this.m_driveSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.fieldOrientation = fieldOrientation;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if(!fieldOrientation.getAsBoolean()){
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        -m_translationXSupplier.getAsDouble(),
                        -m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_driveSubsystem.getGyroscopeRotation()
                        
                )
            );
        }
        else{
            m_driveSubsystem.drive(
                new ChassisSpeeds(
                    m_translationXSupplier.getAsDouble(),
                    m_translationYSupplier.getAsDouble(),
                    m_rotationSupplier.getAsDouble())
                );
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}