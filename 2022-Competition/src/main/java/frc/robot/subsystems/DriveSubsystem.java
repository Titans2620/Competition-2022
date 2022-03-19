package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /**********************************************
  The drive subsystem controls the movement of the robot. This consists of 8 motors and 4 encoders. As well as a gyroscope. 
  To get a better idea of what a swerve drive accomplishes watch the following video. Note: They do not use the same type of drive as we are so the code will differ but the concept will be the same.
  
  https://www.youtube.com/watch?v=0Xi9yb1IMyA
  *********************************************/

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() * SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  //private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);
  

  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(m_kinematics, new Rotation2d(0));

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

  private LimelightSubsystem m_LimeLightSubsystem;
  private ColorSensorSubsystem m_ColorSensorSubsystem;

  public DriveSubsystem(LimelightSubsystem m_LimeLightSubsystem, ColorSensorSubsystem m_ColorSensorSubsystem) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0), Mk3SwerveModuleHelper.GearRatio.STANDARD, FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET);

        m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),Mk3SwerveModuleHelper.GearRatio.STANDARD, FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);

        m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),Mk3SwerveModuleHelper.GearRatio.STANDARD, BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);

        m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),Mk3SwerveModuleHelper.GearRatio.STANDARD, BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);
  
        m_pigeon.setYaw(0);
        resetOdometry(new Pose2d());

        this.m_LimeLightSubsystem = m_LimeLightSubsystem;
        this.m_ColorSensorSubsystem = m_ColorSensorSubsystem;
}

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
        public void zeroGyroscope() {
                m_pigeon.setYaw(0);
        }

        public Rotation2d getGyroscopeRotation() {
                double angle = m_pigeon.getYaw() + 180;
                if(angle > 180)
                        angle -= 360;
                else if(angle < -180)
                        angle += 360;
                return Rotation2d.fromDegrees(angle);

        }

        public Pose2d getPose(){
                return odometer.getPoseMeters();
        }

        public double getHeading(){
                return Math.IEEEremainder(m_pigeon.getYaw(), 360);
        }

        public Rotation2d getRotation2d(){
                return Rotation2d.fromDegrees(getHeading());
        }

        public void resetOdometry(Pose2d pose){
                odometer.resetPosition(pose, getRotation2d());
        }

        public void stopModules(){
                m_frontLeftModule.set(0, m_frontLeftModule.getSteerAngle());
                m_frontRightModule.set(0, m_frontRightModule.getSteerAngle());
                m_backLeftModule.set(0, m_backLeftModule.getSteerAngle());
                m_backRightModule.set(0, m_backRightModule.getSteerAngle());
        }
        public void setModuleStates(SwerveModuleState[] states){
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

                this.drive(m_kinematics.toChassisSpeeds(states[0], states[1], states[2], states[3]));
                states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                odometer.update(getRotation2d(), states);

        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        public void limelightDrive(double xSpeed, double ySpeed, double m_rotation, String allianceColor){
                m_LimeLightSubsystem.setLimelightCamMode("Search");
                m_LimeLightSubsystem.setLimelightLED("on");
                String tableState = m_LimeLightSubsystem.getLimelightState();
                String lastStateWhenNotFound = "FASTLEFT";
                        switch(tableState){
                        case "NOT FOUND":
                                //If the limelight does not find the reflective tape we will rotate to attempt to find it. 
                                //Note: This will result in a spinning motion if there is an issue with the Limelight detecting the reflective tape.
                                /*
                                if(lastStateWhenNotFound == "FASTRIGHT"){
                                m_rotation = Constants.LIMELIGHT_SEARCH_SPEED;
                                }
                                else{
                                m_rotation = -Constants.LIMELIGHT_SEARCH_SPEED;
                                }
                                */
                                break;
                        case "FASTLEFT":
                                //The reflective tape is too far to the left so a CCW (Counterclockwise) rotation will be needed to center the shooter.
                                m_rotation = Constants.LIMELIGHT_FAST_SPEED;
                                lastStateWhenNotFound = "FASTLEFT";
                                break;
                        case "SLOWLEFT":
                                m_rotation = Constants.LIMELIGHT_SLOW_SPEED;
                                break;
                        case "FASTRIGHT":
                                //The reflective tape is too far to the right so a CCW (Clockwise) rotation will be needed to center the shooter.
                                m_rotation = -Constants.LIMELIGHT_FAST_SPEED;
                                lastStateWhenNotFound = "FASTRIGHT";
                                break;
                        case "SLOWRIGHT":
                                m_rotation = -Constants.LIMELIGHT_SLOW_SPEED;
                                break;
                        case "STOP":
                                m_rotation = 0.0;
                                break;
                        default:
                                m_rotation = 0.0;
                                System.out.println("The Limelight State is an invalid value, Valid states are: NOT FOUND, FASTLEFT, FASTRIGHT, SLOWLEFT, SLOWRIGHT, and STOP. The current state is: " + tableState);
                        }
                
                
                        this.drive(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed,
                                        ySpeed,
                                        m_rotation * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                        this.getGyroscopeRotation()
                                )
                        );
                
                        }

        public void setStartingPose(double xCoordinate, double yCoordinate, double degrees){
                odometer.resetPosition(new Pose2d(xCoordinate, yCoordinate, new Rotation2d(Math.toRadians(degrees))), new Rotation2d(Math.toRadians(degrees)));
                m_pigeon.setYaw(degrees);
        }
        @Override
        public void periodic() {
                states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                odometer.update(getRotation2d(), states);
                setModuleStates(states);
                SmartDashboard.putString("Pose", getPose().toString());
    
        }
}