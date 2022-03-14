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
    public static final int[] LEFT_MOTOR_PORTS = {3, 4, 5}; // {12,13,14};
    public static final int[] RIGHT_MOTOR_PORTS = {0, 1, 2}; //{1,2,3};

    public static final int LEFT_JOYSTICK_PORT = 1;
    public static final int RIGHT_JOYSTICK_PORT = 0;
    public static final int LEFT_OPERATOR_JOYSTICK_PORT =  4;
    public static final int RIGHT_OPERATOR_JOYSTICK_PORT = 5;

    // 1 is a linear relationship between joystick position and speed
    // Higher values yield more precision in lower speeds and less in higher speeds
    public static final double MOVEMENT_SENSITIVITY = 3;   //Shaping sensitivity for the forward/backwards Joystick
    public static final double ARCADE_TURNING_SENSITIVITY = 2;   //Shaping sensitivity for the arcade drive turning movement
    public static final double RADIUS_TURNING_SENSITIVITY = 2;
    public static final double MOVEMENT_DEADZONE = 0.15; // Range of joystick values that are treated like zero
    public static final double TURNING_DEADZONE = 0.08; // Range of joystick values that are treated like zero (was at 0.05)
    public static final double DECELERATION_DEADZONE = 0.00;

    // timeout in ms
    public static final int TIMEOUT = 0;
    public static final int SLOT_IDX = 0;
    public static final int PID_IDX = 0;

    public static final double VELOCITY_MULTIPLIER = 22598; //m/s

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

    //HybridDrive constants
    public static final double AUTO_AIM_DEADZONE = 3.0/27;  //Hybrid Drive Graph:
    public static final double AUTO_AIM_SENSITIVITY = 0.420;  //https://www.desmos.com/calculator/mzhsaym2ef  (CHECK GRAPH BEFORE CHANGING CONSTANTS)

    public static final double FALCON_NU = 2048; // the number of native units per rotation

    public static final double K_CARPET = 0.05;
    public static final double K_CARPET_BACK = -0.05;

    public static class Cannon {
        public static final int PORT_TOP = 9;
        public static final int PORT_BOTTOM = 10;

        public static final double KP = 0.05;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KF = 0.0475;

        public static final int SOLENOID_PORT = 7;
    }
    public static class Climber {
        public static final int PORT_LEFT_1 = 11;
        public static final int PORT_LEFT_2 = 12;
        public static final int PORT_LEFT_ROTATE = 13;
        public static final int PORT_RIGHT_1 = 14;
        public static final int PORT_RIGHT_2 = 15;
        public static final int PORT_RIGHT_ROTATE = 16;

        public static final int FIXED_MANUAL_CLIMB_JOYSTICK_PORT = 5;   //Y axis
        public static final double FIXED_MANUAL_CLIMB_JOYSTICK_DEADZONE = 0.12;
        public static final int ROTATING_MANUAL_CLIMB_JOYSTICK_PORT = 5;    //X axis
        public static final double ROTATING_MANUAL_CLIMB_JOYSTICK_DEADZONE = 0.12;
        public static final int ROTATING_ANGLE_JOYSTICK_PORT = 4;   //X axis
        public static final double ROTATING_ANGLE_JOYSTICK_DEADZONE = 0.12;

        public static final int DIO_LS_LEFT_1 = 0; // TODO change
        public static final int DIO_LS_LEFT_2 = 1; // TODO change
        public static final int DIO_LS_RIGHT_1 = 2; // TODO change
        public static final int DIO_LS_RIGHT_2 = 3; // TODO change

        public static final int DEPLOY_RIGHT_POS = 143000; //140000;
        public static final int DEPLOY_LEFT_POS = 145000;

        public static final int RETRACT_POS = 4000; // TODO change

        public static final double KF = 0.051; 
        //To be or not to be that is the question
        public static final double KP_1 = 0.025;
        public static final double KI_1 = 0;
        public static final double KD_1 = 0;
        //Whether tis nobler in the mind to suffer the slings and arrows of outrageous fortunes
        public static final double KP_2 = 0.025;
        public static final double KI_2 = 0;
        public static final double KD_2 = 0;
        //Or to take arms against a sea of sorrows and by opposing end them
        public static final double KP_ROTATE = 0;
        public static final double KI_ROTATE = 0;
        public static final double KD_ROTATE = 0;
        //To die to sleep no more and by a sleep to say we end the thousand natural shocks that flesh is heir to
        public static final double DEPLOY_ACCELERATION = 100000;
        public static final double DEPLOY_CRUISE_VELOCITY = 17500;
        //Tis a consummation devoutly to be wished
        public static final double RETRACT_ACCELERATION = 200000;
        public static final double RETRACT_CRUISE_VELOCITY = 10000;
    }

    public static class DriveTrain {
        public static final double KS = 0.61405;
        public static final double KV = 2.143;
        public static final double KA = 0.25872;
        //To die to sleep to sleep perchance to dream Aye there's the rub
        public static final double KF_RIGHT = 0.046222;
        public static final double P_RIGHT = 0.127; 
        public static final double I_RIGHT = 0;
        public static final double D_RIGHT = 0;
        public static final double AFF_RIGHT = 0.05047;

        public static final double KF_LEFT = 0.046135;
        public static final double P_LEFT = 0.1128;
        public static final double I_LEFT = 0;
        public static final double D_LEFT = 0;
        public static final double AFF_LEFT = 0.05117;  //Benjamin Edward Alex Jones Shapiro Brooder estaba aqui

        public static final double WHEEL_GAP = 0.85162; // distance between wheels in meters
        public static final double WHEEL_RADIUS = Units.inches2Meters(1.97775696); // wheel radius in meters

        public static final double DRIVE_GEAR_RATIO = 20.0 / 3;
        //Ben will not notice the other 5 comments I have left in the Constants class
        public static final double MAX_VELOCITY = 2; 
        public static final double MAX_ACCELERATION = 0.75;

        public static final double AUTO_B = 2; 
        public static final double AUTO_ZETA = 1;
    }

    public static class Intake {
        public static final int PORT_INTAKE = 6;
        public static final int PORT_GRABBER = 7;
        public static final int PORT_LOADER = 8;

        public static final int DIO_SENSOR_1 = 8;
        public static final int DIO_SENSOR_2 = 9;

        public static final int INTAKE_SOLENOID_PORT = 8;
        public static final int INTAKE_SOLENOID_PORT2 = 15;

        public static final double INTAKE_SPEED = 0.75;
        public static final double GRABBER_SPEED = 0.4;
        public static final double LOADER_SPEED = 0.15;
    }

    public static class Limelight {
        public static final int REFLECTIVE_TAPE_PIPELINE = 0;

        public static final double SHOOTER_HEIGHT = 0;
        public static final double SHOOTER_ANGLE = 44 * TAU/360;
        public static final double HUB_HEIGHT = 0;

        public static final double INTAKE_HEIGHT = 0;
        public static final double INTAKE_ANGLE = 0;
        public static final double BALL_HEIGHT = 0;
    }

    //Declares the Button Ports    IMPORTANT: Buttons start at index 1
    public static class Buttons{
        public static final double DEBOUNCE_VALUE = 0;

        //Ports for Intake
        public static final int DEPLOY_INTAKE = 10;
        public static final int RUN_INTAKE = 1;
        public static final int STOP_INTAKE = 2;

        //Ports for Ejecting balls of the wrong color
        public static final int EJECT_FRONT = 4;
        public static final int EJECT_BACK = 6;
        public static final int PUKE = 3;

        //Ports for Shooting
        public static final int SHOOT = 9;
        public static final int RUN_SHOOTER = 8;
        public static final int STOP_SHOOTER = 5;
        public static final int SHOOTER_ANGLE = 7;

        //Ports for static climber
        public static final int FIXED_CLIMB_DEPLOY = 3;
        public static final int FIXED_CLIMB = 2;
        public static final int FIXED_CLIMB_GRAB = 1;
        public static final int FIXED_CLIMB_RELEASE = 11;
    }
}
