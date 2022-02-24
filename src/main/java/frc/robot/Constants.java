/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.lib.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //the ports on the CAN bus for the left and right motors respectively on 2020 robot
    //index 0 should be master motor
    public static final int[] LEFT_MOTOR_PORTS = {12,13,14};// {3, 4, 5}; //;
    public static final int[] RIGHT_MOTOR_PORTS = {1,2,3};//  {0, 1, 2}; //;

    //the ports on the CAN bus for the left and right motors respectively on 2019 robot
    //index 0 should be master motor
    public static final int[] LEFT_MOTOR_PORTS_2019 = {0, 1, 2};
    public static final int[] RIGHT_MOTOR_PORTS_2019 = {12, 13, 14};
    public static final int HDRIVEPORT = 11; //Hopefully the right port

    public static final int[] GRABBER_MOTOR_PORTS = {};
    public static final int[] INTAKE_MOTOR_PORTS = {};
    public static final int[] LOADER_MOTOR_PORTS = {};
    
    //port numbers for various moving parts
    public static final int TURRET_PORT = 4;
    public static final int WINCH_PORT = 18;

    public static final int[] SOLENOID_CHANNELS = {};

    public static final int LEFT_JOYSTICK_PORT = 1;
    public static final int RIGHT_JOYSTICK_PORT = 0;
    public static final int LEFT_OPERATOR_JOYSTICK_PORT = 4;
    public static final int RIGHT_OPERATOR_JOYSTICK_PORT = 5;

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
    public static final double KF = 0.0457; //0.0457
    public static final double RADIUS_DRIVE_MULTIPLIER = 1.8;   //1.3
    
    //Tip control parameters
    public static final double WIDTH = 0.6; //Metric units
    public static final double HEIGHT = 0.3; //center of mass
    public static final double GRAVITY = 9.8; //m/s
    public static final double M2NUCONVERSION = 3564.6; //conversion factor for ticks/100ms to m/s
    public static final double METRICMAX = VELOCITY_MULTIPLIER/M2NUCONVERSION;

    //Max acceleration parameters
    public static final double MAX_ACCEL = 0.0007; //0.001;
    //Also known as PARTTDOTRV; Positive acceleration relative to the direction of the robot's velocity
    public static final double MAX_DECEL = 0.0005; //0.0007;
    //Also known as NARTTDOTRV; Negative acceleration relative to the direction of the robot's velocity
    public static final double MAX_ACCEL_TURN = 0.001;
    //Also known as PAARTTDOTRAAV; Positive angular acceleration relative to the direction of the robot's angular velocity
    public static final double MAX_DECEL_TURN = 0.01;
    //Also known as NAARTTDOTRAAV; Negative angular acceleration relative to the direction of the robot's angular velocity

    //Use this as the superior circle constant
    public static final double TAU = 2 * Math.PI;
    
    //Change the autonomous settings
    public static final double MIN_DISTANCE_AUTO = 1; //Tells at what distance from the target where the robot will stop
    public static final double SPEED_THRESHOLD_AUTO = 2.5; //The distance from the targget at which the robot will not go at moveSpeedAuto; past threshold, speed will be proportional to distance of target
    public static final double MOVE_SPEED_AUTO = 0.2; //The max move speed during autonomous

    //HybridDrive constants
    public static final double HYBRID_DRIVE_DEADZONE = 3.0/27;  //Hybrid Drive Graph:
    public static final double HYBRID_DRIVE_SENSITIVITY = 0.420;  //https://www.desmos.com/calculator/ojy5ecv8fd  (CHECK GRAPH BEFORE CHANGING CONSTANTS)

    public static final String PHOTONVISION_NICKNAME = "cargodetectooor"; // TODO

    public static final double WHEEL_GAP = Units.inches2Meters(21); // distance between wheels in meters
    public static final double WHEEL_RADIUS = 0.085; // wheel radius in meters

    public static final double DRIVE_GEAR_RATIO = 9.65;

    public static final double FALCON_NU = 2048; // the number of native units per rotation

    public static class Cannon {
        public static final int PORT_TOP = 9;
        public static final int PORT_BOTTOM = 10;

        public static final double KP = 0.05;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KF = 0.0475;
    }
    public static class Climber {
        public static final int PORT_LEFT_1 = 11;
        public static final int PORT_LEFT_2 = 12;
        public static final int PORT_LEFT_ROTATE = 13;
        public static final int PORT_RIGHT_1 = 14;
        public static final int PORT_RIGHT_2 = 15;
        public static final int PORT_RIGHT_ROTATE = 16;

        public static final int DIO_LS_LEFT_1 = 0; // TODO change
        public static final int DIO_LS_LEFT_2 = 1; // TODO change
        public static final int DIO_LS_RIGHT_1 = 2; // TODO change
        public static final int DIO_LS_RIGHT_2 = 3; // TODO change

        public static final double KF = 0.0457;

        public static final double KP_1 = 0;
        public static final double KI_1 = 0;
        public static final double KD_1 = 0;
        
        public static final double KP_2 = 0;
        public static final double KI_2 = 0;
        public static final double KD_2 = 0;

        public static final double KP_ROTATE = 0;
        public static final double KI_ROTATE = 0;
        public static final double KD_ROTATE = 0;
    }

    public static class DriveTrain {
        public static final double KS = 0.6613;
        public static final double KV = 1.8944;
        public static final double KA = 0.29877;

        public static final double MAX_VELOCITY = .2; //max speed in meters per second
        public static final double MAX_ACCELERATION = .2; //max speed in meters per second per second
    }

    public static class Intake {
        public static final int PORT_INTAKE = 6;
        public static final int PORT_GRABBER = 7;
        public static final int PORT_LOADER = 8;

        public static final int DIO_SENSOR_1 = 4;
        public static final int DIO_SENSOR_2 = 5;
    }

}
