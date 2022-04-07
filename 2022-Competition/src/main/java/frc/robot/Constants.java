package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5334;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6604;

    //FRONT LEFT MODULE//
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(240);

    //FRONT RIGHT MODULE//
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(186);

    //BACK LEFT MODULE//
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 8;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(168);

    //BACK RIGHT MODULE//
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(274);

    //-----CAN-----//
    public static final int INTAKE_ROLLER = 13;
    public static final int INTAKE_ROTATE = 14;
    public static final int FEED_WHEEL = 15;
    public static final int SHOOTER_WHEEL = 16;
    public static final int LEFT_CLIMB_EXTEND = 17;
    public static final int RIGHT_CLIMB_EXTEND = 18;
    public static final int DRIVETRAIN_PIGEON_ID = 19;


    //-----PWM-----//
    public static final int PWM_CLIMB_PIVOT = 0;

    //---DIO Sensors---//
    public static final int INTAKE_LOWER_ROTATE_LIMIT = 0;
    public static final int INTAKE_UPPER_ROTATE_LIMIT = 1;
    public static final int CLIMB_LEFT_LIMIT = 2;
    public static final int CLIMB_RIGHT_LIMIT = 3;
    public static final int LINESENSOR = 4;

    //LED CODE//
    public static final int LED = 9;
    public static final int LEDLENGTH = 32;

    //OPERATOR SPEEDS//
    public static final double INTAKESPEED = -1.0;
    public static final double INTAKEROTATEUPSPEED = .5;
    public static final double INTAKEROTATEDOWNSPEED = -.25;
    public static final double FEEDSPEED = .25; //Originally .25
    public static final double OUTFEEDSPEED = -.5;
    public static final double SHOOTERSPEED = .72;
    public static final double SHOOTEROUTFEEDSPEED = -.5;
    public static final double LOWGOALSHOOTERSPEED = .42;

    //LIMELIGHT DRIVE SPEEDS//
    public static final double LIMELIGHT_SLOW_SPEED = 0.1;
    public static final double LIMELIGHT_FAST_SPEED = 0.25;
    public static final double LIMELIGHT_SEARCH_SPEED = 0.5;
    public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 7.0;
    public static final double LIMELIGHT_TARGET_HEIGHT_INCHES = 102.0; //8'6"
    public static final double LIMELIGHT_LENSE_FLOOR_HEIGHT = 26.75;
    //CONSTANT LIMELIGHT STOP//
    public static final String LIMELIGHT_STOP = "STOP";

    public static final int SHOOTER_MAX_RPM = 5100;
    public static final double SHOOTER_MIN_SPEED_PERCENT = 0.680;
    public static final double SHOOTER_MAX_SPEED_PERCENT = 0.830;
    public static final int SHOOTER_MIN_DISTANCE_INCHES = 120;
    public static final int SHOOTER_MAX_DISTANCE_INCHES = 250;
    //CLIMB SPEEDS//
    public static final double CLIMB_DOWN_SPEED = -1;
    public static final double CLIMB_UP_SPEED = 1;
    public static final double CLIMB_PIVOT_FORWARD_SPEED = .50;
    public static final double CLIMB_PIVOT_BACK_SPEED = -.50;

    //BASIC MOVEMENT VARIANCE THRESHOLD//
    public static final double BASIC_MOVEMENT_VARIANCE_THRESHOLD = 0.1; 

    //WRONG BALL AIM VARIANCE/
    public static final double WRONG_BALL_AIM_VARIANCE_PIXELS = 15;

    public static final class AutoConstants {
        /*
            -----Constants for Autonomous Mode ONLY-----
        */
        public static final double kMaxSpeedMetersPerSecond = 1.25;
        public static final double kMaxAngularSpeedRadiansPerSecond = (4 * Math.PI) / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond * 10,
                        kMaxAngularAccelerationRadiansPerSecondSquared * 10);

        public static final double BASIC_ROTATION_FAST_SPEED = 0.9;
        public static final double BASIC_ROTATION_SLOW_SPEED = 0.4;
    }
}

