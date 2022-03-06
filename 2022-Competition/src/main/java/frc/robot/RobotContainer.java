package frc.robot;

import java.util.List;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmRotateDefaultCommand;
import frc.robot.commands.ArmRotateIntakeCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.ClimbDefaultCommand;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeInfeedCommand;
import frc.robot.commands.IntakeManualCommand;
import frc.robot.commands.IntakeShootCommand;
import frc.robot.commands.LimelightDefaultCommand;
import frc.robot.commands.LimelightGetStateCommand;
import frc.robot.commands.LimelightSearchCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.ShooterManualShootCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.commands.Autonomous.AutonomousTaxiCommandGroup;
import frc.robot.commands.Autonomous.AutonomousTaxiShootCommandGroup;
import frc.robot.commands.DriveLimelightCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ColorSensorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem(); 
  private final ColorSensorSubsystem m_ColorSensorSubsystem = new ColorSensorSubsystem(); 

  private final XboxController m_driveController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  NetworkTableEntry isRedAlliance;
  private final AutonomousTaxiCommandGroup taxi = new AutonomousTaxiCommandGroup(m_driveSubsystem);
  private final AutonomousTaxiShootCommandGroup taxiShoot = new AutonomousTaxiShootCommandGroup(m_driveSubsystem, m_intakeSubsystem, m_ShooterSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {

      CameraServer.startAutomaticCapture();

      m_driveSubsystem.setDefaultCommand(new DriveDefaultCommand( // Drive //
              m_driveSubsystem,
              () -> modifyAxis(m_driveController.getRawAxis(0)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> -modifyAxis(m_driveController.getRawAxis(1)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> modifyAxis(m_driveController.getRawAxis(4)) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
              () -> m_driveController.getRawButton(7)
      ));

  
      m_intakeSubsystem.setDefaultCommand(new IntakeDefaultCommand(m_intakeSubsystem));
      //m_climbSubsystem.setDefaultCommand(new ClimbDefaultCommand(m_climbSubsystem));
      m_ShooterSubsystem.setDefaultCommand(new ShooterDefaultCommand(m_ShooterSubsystem));
      m_ArmSubsystem.setDefaultCommand(new ArmRotateDefaultCommand(m_ArmSubsystem));
      m_limelightSubsystem.setDefaultCommand(new LimelightDefaultCommand(m_limelightSubsystem));

  

      m_chooser.setDefaultOption("Taxi", taxi);
      m_chooser.addOption("Taxi and Shoot", taxiShoot);
      
      // Configure the button bindings
      configureButtonBindings();

      // Smart Dashboard
      putSmartdashboard();
  }

  private void configureButtonBindings() {

    new JoystickButton(m_driveController, 8).whenPressed(()-> m_driveSubsystem.zeroGyroscope());
    new JoystickButton(m_operatorController, 6).whenHeld(new ParallelCommandGroup(new DriveLimelightCommand(m_driveSubsystem, m_limelightSubsystem, () -> modifyAxis(m_driveController.getRawAxis(0)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, () -> -modifyAxis(m_driveController.getRawAxis(1)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, m_ColorSensorSubsystem, getAlliance(), () -> modifyAxis(m_driveController.getRawAxis(4)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
      new ShooterShootCommand(m_ShooterSubsystem, m_limelightSubsystem),
        new LimelightSearchCommand(m_limelightSubsystem),
          new IntakeShootCommand(m_intakeSubsystem, m_ShooterSubsystem, m_ColorSensorSubsystem, m_limelightSubsystem, getAlliance())));
    if(!m_operatorController.getRawButton(2) || !m_operatorController.getRawButton(3))
        new JoystickButton(m_operatorController, 1).whenHeld(new ParallelCommandGroup(new IntakeInfeedCommand(m_intakeSubsystem, m_ColorSensorSubsystem), new ArmRotateIntakeCommand(m_ArmSubsystem)));
    new JoystickButton(m_operatorController, 2).whenHeld(new ArmRotateCommand(m_ArmSubsystem, true));
    new JoystickButton(m_operatorController, 3).whenHeld(new ArmRotateCommand(m_ArmSubsystem, false));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return m_chooser.getSelected();
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

  public String getAlliance(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("FMSInfo");
    isRedAlliance = table.getEntry("IsRedAlliance");
    if(isRedAlliance.getBoolean(true)){
      return "red";
    }
    else{
      return "blue";
    }
  }

  public void putSmartdashboard(){
    SmartDashboard.putBoolean("Is Red Alliance", isRedAlliance.getBoolean(true));
    SmartDashboard.putData("Choose Autonomous Mode", m_chooser);
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}