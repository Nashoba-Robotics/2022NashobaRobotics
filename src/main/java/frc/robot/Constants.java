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
    //If you put a motor w/ a limited axis of rotation change the time condition in the Diagnostic Command
    public static final int[] MISC_MOTOR_PORTS = {};    

    public static final int[] SOLENOID_CHANNELS = {};

    public static final int LEFT_JOYSTICK_PORT = 1;
    public static final int RIGHT_JOYSTICK_PORT = 0;

    // 1 is a linear relationship between joystick position and speed
    // Higher values yield more precision in lower speeds and less in higher speeds
    public static final int MOVEMENT_SENSITIVITY = 2;   //Shaping sensitivity for the forward/backwards Joystick
    public static final int TURN_SENSITIVITY = 4;   //Shaping sensitivity for the turning movement

    // timeout in ms
    public static final int TIMEOUT = 30;
    public static final int PID_LOOP_IDX = 0;

    public static final double VELOCITY_MULTIPLIER = 19000;

    // motor constants
    public static double KP = 0.0496;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.0496;
}
