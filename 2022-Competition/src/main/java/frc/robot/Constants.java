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

    public static final int DRIVETRAIN_PIGEON_ID = 19;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(149.05);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(99.39);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 8;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(80);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(55);

    public static final int INTAKE_ROLLER = 13;
    public static final int INTAKE_ROTATE = 14;
    public static final int FEED_WHEEL = 15;
    public static final int SHOOTER_WHEEL = 16;
    public static final int LEFT_CLIMB = 17;
    public static final int RIGHT_CLIMB = 18;

    //Sensors//

    public static final int INTAKE_ROTATE_LIMIT = 0;
    public static final int CLIMB_LOW_LIMIT = 1;
    public static final int CLIMB_HIGH_LIMIT = 2;


    //SPEEDS//
    public static final double INTAKESPEED = -1.0;
    public static final double INTAKEROTATEUPSPEED = .25;
    public static final double INTAKEROTATEDOWNSPEED = -.25;
    public static final double FEEDSPEED = .2;
    public static final double SHOOTERSPEED = .5;

    public static final double LIMELIGHT_SLOW_SPEED = 0.1;
    public static final double LIMELIGHT_FAST_SPEED = 0.25;
    public static final double LIMELIGHT_SEARCH_SPEED = 0.3;

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1.25;
        public static final double kMaxAngularSpeedRadiansPerSecond = (4 * Math.PI) / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }
}

