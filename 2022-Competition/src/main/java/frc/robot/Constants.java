// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int kLeftController = 0;
    public static final int kRightController = 1;

    public static final double ROBOT_LENGTH = 2;
    public static final double ROBOT_WIDTH = 2;

     public static final int kFrontLeftDriveMotorPort = -1;
    public static final int kRearLeftDriveMotorPort = -1;
    public static final int kFrontRightDriveMotorPort = -1;
    public static final int kRearRightDriveMotorPort = -1;

    public static final int kFrontLeftTurningMotorPort = -1;
    public static final int kRearLeftTurningMotorPort = -1;
    public static final int kFrontRightTurningMotorPort = -1;
    public static final int kRearRightTurningMotorPort = -1;

    public static final int[] kFrontLeftTurningEncoderPorts = new int[] {-1, -1};
    public static final int[] kRearLeftTurningEncoderPorts = new int[] {-1, -1};
    public static final int[] kFrontRightTurningEncoderPorts = new int[] {-1, -1};
    public static final int[] kRearRightTurningEncoderPorts = new int[] {-1, -1};

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final int[] kFrontLeftDriveEncoderPorts = new int[] {-1, -1};
    public static final int[] kRearLeftDriveEncoderPorts = new int[] {-1, -1};
    public static final int[] kFrontRightDriveEncoderPorts = new int[] {-1, -1};
    public static final int[] kRearRightDriveEncoderPorts = new int[] {-1, -1};

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final double kTrackWidth = 0.5;
}
