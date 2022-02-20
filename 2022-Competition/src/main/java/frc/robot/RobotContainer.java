package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmRotateDefaultCommand;
import frc.robot.commands.ArmRotateManualCommand;
import frc.robot.commands.ClimbDefaultCommand;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeManualCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.DriveAuto1Command;
import frc.robot.commands.LimelightDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

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
  //private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  //private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();  

  private final Joystick m_controller = new Joystick(0);

  //private final XboxController m_drivController = new XboxController(0);
  //private final XboxController m_operatorController = new XboxController(1);

  //CameraServer camera = new ();

  //SendableChooser<Command> m_chooser = new SendableChooser<>();

  //private final DriveAuto1Command auto1 = new DriveAuto1Command(m_driveSubsystem);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
      // Set up the default command for the drivetrain.
      // The controls are for field-oriented driving: 
      // Left stick Y axis -> forward and backwards movement
      // Left stick X axis -> left and right movement
      // Right stick X axis -> rotation

      CameraServer.startAutomaticCapture();

      m_driveSubsystem.setDefaultCommand(new DriveDefaultCommand( // Drive //
              m_driveSubsystem,
              () -> modifyAxis(m_controller.getX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> -modifyAxis(m_controller.getY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> modifyAxis(m_controller.getZ()) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
      ));

  
      m_intakeSubsystem.setDefaultCommand(new IntakeDefaultCommand(m_intakeSubsystem));
      //m_climbSubsystem.setDefaultCommand(new ClimbDefaultCommand(m_climbSubsystem));
      //m_ShooterSubsystem.setDefaultCommand(new ShooterDefaultCommand(m_ShooterSubsystem));
      m_ArmSubsystem.setDefaultCommand(new ArmRotateDefaultCommand(m_ArmSubsystem));
  
      //m_chooser.setDefaultOption("Test Auto", auto1);
      
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

    new JoystickButton(m_controller, 10).whenPressed(()-> m_driveSubsystem.zeroGyroscope());
    //new JoystickButton(m_controller, 1).whenPressed(new LimelightDriveCommand(m_driveSubsystem, m_limelightSubsystem, () -> -m_controller.getX(), () -> -m_controller.getY()));
    new JoystickButton(m_controller, 2).whenHeld(new IntakeManualCommand(m_intakeSubsystem));
    new JoystickButton(m_controller, 12).whileHeld(new ArmRotateManualCommand(m_ArmSubsystem, m_controller.getRawButton(11), m_controller.getRawButton(16)));
    new JoystickButton(m_controller, 12).whileHeld(new ArmRotateManualCommand(m_ArmSubsystem, m_controller.getRawButton(11), m_controller.getRawButton(16)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      //return m_chooser.getSelected();
      return new InstantCommand();

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
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}