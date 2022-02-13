package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.LimelightDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  //private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  
  private final Joystick m_controller = new Joystick(0);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving: 
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    
    m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand( // Drive //
            m_driveSubsystem,
            () -> -m_controller.getX(),
            () -> -m_controller.getY(),
            () -> -m_controller.getTwist()
    ));
  
   /* m_intakeSubsystem.setDefaultCommand(new RunCommand( // intake //
      () -> m_intakeSubsystem.intake(m_controller.getRawButton(2), m_controller.getRawButton(11), m_controller.getRawButton(16)), m_intakeSubsystem));
    */
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      new JoystickButton(m_controller, 1).whenPressed(()-> m_driveSubsystem.zeroGyroscope());
      new JoystickButton(m_controller, 1).whenPressed(new LimelightDriveCommand(m_driveSubsystem, () -> -m_controller.getX(), () -> -m_controller.getY()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxSpeedMetersPerSecond).setKinematics(m_driveSubsystem.m_kinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)),
       List.of(
          new Translation2d(1,0),
          new Translation2d(1, -1)),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig);

      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);

      ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory, 
        m_driveSubsystem::getPose, 
        m_driveSubsystem.m_kinematics,
        xController, 
        yController, 
        thetaController, 
        m_driveSubsystem::setModuleStates,
        m_driveSubsystem);

        return new SequentialCommandGroup(new InstantCommand(() -> m_driveSubsystem.resetOdometry(trajectory.getInitialPose())), swerveControllerCommand, new InstantCommand(() -> m_driveSubsystem.stopModules()));

      }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}