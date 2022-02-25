package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmRotateDefaultCommand;
import frc.robot.commands.ArmRotateIntakeCommand;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.ClimbDefaultCommand;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeInfeedCommand;
import frc.robot.commands.IntakeManualCommand;
import frc.robot.commands.LimelightDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.ShooterManualShootCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.commands.DriveAuto1Command;
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

  //private final Joystick m_controller = new Joystick(0);

  private final XboxController m_driveController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  NetworkTableEntry isRedAlliance;
  //private final DriveAuto1Command auto1 = new DriveAuto1Command(m_driveSubsystem);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {

      CameraServer.startAutomaticCapture();

      m_driveSubsystem.setDefaultCommand(new DriveDefaultCommand( // Drive //
              m_driveSubsystem,
              () -> modifyAxis(m_driveController.getRawAxis(0)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> -modifyAxis(m_driveController.getRawAxis(1)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> modifyAxis(m_driveController.getRawAxis(4)) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
      ));

  
      m_intakeSubsystem.setDefaultCommand(new IntakeDefaultCommand(m_intakeSubsystem));
      //m_climbSubsystem.setDefaultCommand(new ClimbDefaultCommand(m_climbSubsystem));
      m_ShooterSubsystem.setDefaultCommand(new ShooterDefaultCommand(m_ShooterSubsystem));
      m_ArmSubsystem.setDefaultCommand(new ArmRotateDefaultCommand(m_ArmSubsystem));
      m_limelightSubsystem.setDefaultCommand(new LimelightDefaultCommand(m_limelightSubsystem));

  

      //m_chooser.setDefaultOption("Test Auto", auto1);
      
      // Configure the button bindings
      configureButtonBindings();

      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      NetworkTable table = inst.getTable("FMSInfo");
      isRedAlliance = table.getEntry("IsRedAlliance");

      // Smart Dashboard
      putSmartdashboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_operatorController, 7).whenPressed(()-> m_driveSubsystem.zeroGyroscope());
    new JoystickButton(m_operatorController, 6).whenHeld(new ShooterManualShootCommand(m_ShooterSubsystem));
    new JoystickButton(m_operatorController, 7).whenHeld(new ParallelCommandGroup(new DriveLimelightCommand(m_driveSubsystem, m_limelightSubsystem, () -> -m_driveController.getRawAxis(0), () -> -m_driveController.getRawAxis(1)), new ShooterShootCommand(m_ShooterSubsystem, m_intakeSubsystem, m_ColorSensorSubsystem, getAlliance())));
    //new JoystickButton(m_controller, 3).whenHeld(new ParallelCommandGroup(new ShooterShootCommand(m_ShooterSubsystem, m_intakeSubsystem), new DriveLimelightCommand(m_driveSubsystem, m_limelightSubsystem, () -> -m_controller.getX(), () -> -m_controller.getY())));
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
    if(isRedAlliance.getBoolean(true)){
      return "red";
    }
    else{
      return "blue";
    }
  }

  public void putSmartdashboard(){
    SmartDashboard.putBoolean("Is Red Alliance", isRedAlliance.getBoolean(true));
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}