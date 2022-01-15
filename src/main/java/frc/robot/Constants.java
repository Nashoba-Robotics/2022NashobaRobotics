/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int[] LEFT_MOTOR_PORTS = {12,13,14};
    public static final int[] RIGHT_MOTOR_PORTS = {1,2,3};

    public static final int[] LEFT_MOTOR_PORTS_2019 = {0, 1, 2};
    public static final int[] RIGHT_MOTOR_PORTS_2019 = {12, 13, 14};
    public static final int HDRIVEPORT = 11; //Hopefully the right port
    
    public static final int TURRET_PORT = 4;
    public static final int WINCH_PORT = 18;

    public static final int[] SOLENOID_CHANNELS = {};

    public static final int LEFT_JOYSTICK_PORT = 1;
    public static final int RIGHT_JOYSTICK_PORT = 0;

    // 1 is a linear relationship between joystick position and speed
    // Higher values yield more precision in lower speeds and less in higher speeds
    public static final double MOVEMENT_SENSITIVITY = 3;   //Shaping sensitivity for the forward/backwards Joystick
    public static final double ARCADE_TURNING_SENSITIVITY = 2;   //Shaping sensitivity for the arcade drive turning movement
    public static final double RADIUS_TURNING_SENSITIVITY = 2;
    public static final double MOVEMENT_DEADZONE = 0.08; // Range of joystick values that are treated like zero
    public static final double TURNING_DEADZONE = 0.05; // Range of joystick values that are treated like zero
    public static final double DECELERATION_DEADZONE = 0.00;

    // timeout in ms
    public static final int TIMEOUT = 0;
    public static final int SLOT_IDX = 0;
    public static final int PID_IDX = 0;

    public static final double VELOCITY_MULTIPLIER = 21136; //m/s
    // Arbitrary feed forward
	public static final double AFF = 0.051;

    // motor constants
    //public static final double KP = 0.0457;
    public static final double KP = 0.12;
    public static final double KI = 0.001;
    public static final double KD = 0.001;
    public static final double KF = 0.0457;
    public static final double RADIUS_DRIVE_MULTIPLIER = 1.3;
    
    //Tip control parameters
    public static final double WIDTH = 0.6; //Metric units
    public static final double HEIGHT = 0.3; //center of mass
    public static final double GRAVITY = 9.8; //m/s
    public static final double M2NUCONVERSION = 3564.6; //conversion factor for ticks/100ms to m/s
    public static final double METRICMAX = VELOCITY_MULTIPLIER/M2NUCONVERSION;

    //Max acceleration parameters
    public static final double MAX_ACCEL = 0.001;
    public static final double MAX_DECEL = 0.0006;
}
