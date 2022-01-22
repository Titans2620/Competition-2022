package frc.robot;

public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 8;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

    public static final int INTAKE_FEED = 13;
    public static final int INTAKE_ROTATE = 14;
    public static final int INNER_FEED = 15;
    public static final int SHOOTER = 16;
    public static final int LEFT_CLIMB = 17;
    public static final int RIGHT_CLIMB = 18;

    //SPEEDS//
    public static final double INTAKESPEED = .25;
    public static final double INTAKEROTATESPEED = .25;
}

