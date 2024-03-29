/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
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
    //To be or not to be that is the question
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
    //Whether tis nobler in the mind to suffer the slings and arrows of outrageous fortune
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
    public static final double AUTO_AIM_DEADZONE = 2.0/27;  //Hybrid Drive Graph:
    public static final double AUTO_AIM_SENSITIVITY = 0.45;  //https://www.desmos.com/calculator/mzhsaym2ef  (CHECK GRAPH BEFORE CHANGING CONSTANTS)

    public static final double FALCON_NU = 2048; // the number of native units per rotation

    public static class Leds {
        public static final int[] DEPLOY_CLIMBER = {146, 100, 71};
        public static final int[] TRAVERSAL_CLIMB = {131, 74, 166};
        public static final int[] PUSH_CLIMBER = {183, 74, 102};
        public static final int[] TEMPORARY_RELEASE = {56, 243, 214};
        public static final int[] MANUAL_PUSH = {215, 154, 216};
        public static final int[] MANUAL_CLIMB = {17, 84, 175};

        public static final int[] TWO_BALLS = {255,0,0};
        public static final int[] ONE_BALL = {0,0,255};
        public static final int[] NO_BALLS = {255,255,255};
    }
    public static class Cannon {
        public static final int PORT_TOP = 9;
        public static final int PORT_BOTTOM = 10;

        public static final double KP = 0.011;
        public static final double KI = 0; //0.0003;
        public static final double KD = 0.5;
        public static final double KF = 0.0475;

        public static final int SOLENOID_PORT = 7;

        public static final double VOLTAGE_COMPENSATION = 10;

        public static final double CLOSE_SHOT_SPEED = 0.47;
        public static final double farShotSpeed(double ty) {
            // return 0.53 - 0.00825 * ty;
            // return 0.543677 - 0.00620598 * ty;

            //return 0.53491 - 0.00686079 * ty + 0.0000779582 * ty*ty;
            return 0.56391 - 0.00796079 * ty + 0.0000879582 * ty*ty;
        }
    }
    public static class Climber {
        public static final int LEFT_PUSHER_PORT = 15; 
        public static final int RIGHT_PUSHER_PORT = 12; //Inverted

        public static final int LEFT_CLIMBER_PORT = 13;
        public static final int RIGHT_CLIMBER_PORT = 16; //Inverted

        public static final int FIXED_MANUAL_CLIMB_JOYSTICK_PORT = 5;   //Y axis
        public static final double FIXED_MANUAL_CLIMB_JOYSTICK_DEADZONE = 0.12;
        public static final int ROTATING_MANUAL_CLIMB_JOYSTICK_PORT = 5;    //X axis
        public static final double ROTATING_MANUAL_CLIMB_JOYSTICK_DEADZONE = 0.12;
        public static final int ROTATING_ANGLE_JOYSTICK_PORT = 4;   //X axis
        public static final double ROTATING_ANGLE_JOYSTICK_DEADZONE = 0.12;

        // public static final int DEPLOY_LEFT_POS = 184_500;
        // public static final int DEPLOY_RIGHT_POS = 181_500;
        public static final int DEPLOY_LEFT_POS = 183_000;
        public static final int DEPLOY_RIGHT_POS = 183_000;
        public static final int DEPLOY_LEFT_PUSHER_POS = -109_000;
        public static final int DEPLOY_RIGHT_PUSHER_POS = -109_000;

        // public static final double DEPLOY_ACCELERATION = 100000;
        // public static final double DEPLOY_CRUISE_VELOCITY = 17500;
        public static final double DEPLOY_LEFT_ACCELERATION = 40_000;
        public static final double DEPLOY_RIGHT_ACCELERATION = 40_000;
        public static final double DEPLOY_LEFT_CRUISE_VELOCITY = 20_000;
        public static final double DEPLOY_RIGHT_CRUISE_VELOCITY = 20_500;

        public static final double DEPLOY_PUSH_ACCELERATION = 20_000;
        public static final double DEPLOY_PUSH_CRUISE_VELOCITY = 8_000;

        public static final int PUSH_LEFT_POS = 19_000;
        public static final int PUSH_RIGHT_POS = 19_000;

        public static final int PUSH_LEFT_CRUISE_VELOCITY = 15_000;
        public static final int PUSH_RIGHT_CRUISE_VELOCITY = 15_000;

        public static final int PUSH_LEFT_ACCELERATION = 15_000;
        public static final int PUSH_RIGHT_ACCELERATION = 15_000;

        public static final int PUSH_DEADZONE = 200;

        // public static final int RETRACT_LEFT_POS = 13_750;
        // public static final int RETRACT_RIGHT_POS = 13_750;
     //   public static final int RETRACT_LEFT_POS = 10_000;
    //    public static final int RETRACT_RIGHT_POS = 10_000;
        public static final int RETRACT_LEFT_POS = 7_500;
        public static final int RETRACT_RIGHT_POS = 7_500;

        // public static final int RELEASE_LEFT_POS = 70_000;
        // public static final int RELEASE_RIGHT_POS = 70_000;
        public static final int RELEASE_LEFT_POS = 160_000;
        public static final int RELEASE_RIGHT_POS = 160_000;

        public static final int RELEASE_LEFT_CRUISE_VELOCITY = 15_000;
        public static final int RELEASE_RIGHT_CRUISE_VELOCITY = 15_000;
        
        public static final int RELEASE_LEFT_ACCELERATION = 20_000;
        public static final int RELEASE_RIGHT_ACCELERATION = 20_000;

        public static final int RELEASE_DEADZONE = 700;

        public static final int RESET_DEADZONE = 500;

        // public static final int RELEASE_LEFT_PUSHER_SLOW_POS = -10_000;
        // public static final int RELEASE_RIGHT_PUSHER_SLOW_POS = -10_000;

        // public static final int RELEASE_LEFT_PUSHER_SLOW_POS = -3_000;
        // public static final int RELEASE_RIGHT_PUSHER_SLOW_POS = -3_000;
        public static final int RELEASE_LEFT_PUSHER_SLOW_POS = 10_500; //-1_750
        public static final int RELEASE_RIGHT_PUSHER_SLOW_POS = 10_500;

        public static final int RELEASE_LEFT_PUSHER_CRUISE_VELOCITY_SLOW = 500;
        public static final int RELEASE_RIGHT_PUSHER_CRUISE_VELOCITY_SLOW = 500;

        public static final int RELEASE_LEFT_PUSHER_ACCELERATION_SLOW = 2000;
        public static final int RELEASE_RIGHT_PUSHER_ACCELERATION_SLOW = 2000;

        public static final int RELEASE_LEFT_PUSHER_FAST_POS = -80_000;
        public static final int RELEASE_RIGHT_PUSHER_FAST_POS = -80_000;

        public static final int RELEASE_LEFT_PUSHER_CRUISE_VELOCITY_FAST = 50_000;
        public static final int RELEASE_RIGHT_PUSHER_CRUISE_VELOCITY_FAST = 50_000;

        public static final int RELEASE_LEFT_PUSHER_ACCELERATION_FAST = 40_000;
        public static final int RELEASE_RIGHT_PUSHER_ACCELERATION_FAST = 40_000;

        public static final double KF_CLIMBER = 0.047;
        public static final double KP_CLIMBER = 0.025;
        public static final double KI_CLIMBER = 0;
        public static final double KD_CLIMBER = 0;

        public static final double KF_PUSHER = 0.047;
        public static final double KP_PUSHER = 0.025;
        public static final double KI_PUSHER = 0;
        public static final double KD_PUSHER = 0;

        public static final int CLIMB_DEADZONE = 200;
        
        public static final double RETRACT_LEFT_ACCELERATION = 20_000;
        public static final double RETRACT_RIGHT_ACCELERATION = 20_000;
        public static final double RETRACT_LEFT_CRUISE_VELOCITY = 10_000;
        public static final double RETRACT_RIGHT_CRUISE_VELOCITY = 10_500;

        public static final int FORWARD_SOFT_LIMIT = 190000;
        public static final int REVERSE_SOFT_LIMIT = 1000;
    }

    public static class DriveTrain {
        public static final double KS = 0.66937;
        public static final double KV = 2.1999;
        public static final double KA = 0.2522;

        public static final double KF_RIGHT = 0.04365; //0.046222;
        public static final double P_RIGHT = 0.04843; //0.127; 
        public static final double I_RIGHT = 0;
        public static final double D_RIGHT = 0;
        public static final double AFF_RIGHT = 0.054; //0.05047; 0.0563

        public static final double KF_LEFT = 0.04374; //0.046135;
        public static final double P_LEFT = 0.045201; //0.1128;
        public static final double I_LEFT = 0;
        public static final double D_LEFT = 0;
        public static final double AFF_LEFT = 0.0575; //0.05117;

        public static final double WHEEL_GAP = 0.85162; // distance between wheels in meters
        // public static final double WHEEL_RADIUS = Units.inches2Meters(1.97775696); // wheel radius in meters
        // public static final double WHEEL_RADIUS = Units.inches2Meters(1.893888); // wheel radius in meters
        public static final double WHEEL_RADIUS = 0.04697; // wheel radius in meters

        public static final double DRIVE_GEAR_RATIO = 20.0 / 3;

        public static final double MAX_VELOCITY = 5; 
        public static final double MAX_ACCELERATION = 2.5;

        public static final double AUTO_B = 2; 
        public static final double AUTO_ZETA = 1;

        public static final double FAR_LEFT_START_ANGLE = -2.268928028;
        public static final double CLOSE_LEFT_START_ANGLE = 6.033;
        public static final double CLOSE_RIGHT_START_ANGLE = 0;
        public static final double FAR_RIGHT_START_ANGLE = 0;

        public static final DifferentialDriveKinematics KINEMATICS = 
        new DifferentialDriveKinematics(WHEEL_GAP);

        public static final double AUTO_AIM_ACCELARATION = 30_000;
    }

    public static class Intake {
        public static final int PORT_INTAKE = 6;
        public static final int PORT_GRABBER = 7;
        public static final int PORT_LOADER = 8;

        public static final int DIO_SENSOR_1 = 8;
        public static final int DIO_SENSOR_2 = 9;

        public static final int INTAKE_SOLENOID_PORT = 8;
        public static final int INTAKE_SOLENOID_PORT2 = 14;

        public static final double INTAKE_SPEED = 0.75;
        public static final double GRABBER_SPEED = 0.6;
        public static final double LOADER_SPEED = 0.15;

        // public static final double LOADER_KF = 0.0475;
        // public static final double LOADER_KP = 0.03;
        // public static final double LOADER_KI = 0;
        // public static final double LOADER_KD = 0.2;

        public static final int COLOR_REJECTION_SWITCH_PORT = 12;

        public static final int COLOR_REJECTION_PUKE_TIME = 3000; // in millis
        public static final int COLOR_REJECTION_SHOOT_TIME = 3000; // in millis
    }

    public static class ColorSensor{
        public static final double D_RED_MAX = 0.06;
        public static final double D_BLUE_MAX = 0.115;

        public static final double[] RED1 = {0.1869, 0.5761, 0.0144, 0.2226};
        public static final double[] RED2 = {0.1672, 0.4643, 0.0093, 0.3592};
        public static final double[] BLUE1 = {0.4749, 0.3935, 0.0009, 0.1225};
        public static final double[] BLUE2 = {0.4138, 0.3661, 0.0077, 0.2123};
    }

    public static class Limelight {
        public static final int REFLECTIVE_TAPE_PIPELINE = 0;

        public static final double SHOOTER_HEIGHT = 0;
        public static final double SHOOTER_ANGLE = 44 * TAU/360;
        public static final double HUB_HEIGHT = 0;

        public static final double INTAKE_HEIGHT = 0;
        public static final double INTAKE_ANGLE = 0;
        public static final double BALL_HEIGHT = 0;

        public static final double AUTO_AIM_OFFSET = 4; // before: 3.5
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
        public static final int SHOOTER_ANGLE = 12;

        //Ports for static climber
        public static final int FIXED_CLIMB_DEPLOY = 3;
        public static final int FIXED_CLIMB = 2;
        public static final int TRAVERSAL_CLIMB = 1;
        public static final int FIXED_CLIMB_RELEASE = 11;

        public static final int ENABLE_MANUAL_PUSH = 6;
        public static final int ENABLE_MANUAL_CLIMB = 5;

        public static final int AUTO_AIM = 1;
    }

    public static class FIELD {
        public static final double ANGLE_OF_RESISTANCE_RED = 0;
        public static final double ANGLE_OF_RESISTANCE_BLUE = 0;

        // public static final double K_CARPET_BLUE = 0.094564;
        // public static final double K_CARPET_RED = 0.094564;
        // public static final double K_CARPET_BLUE = 0.1049;
        // public static final double K_CARPET_RED = 0.1049;
        public static final double K_CARPET_RED = 0.09936;
        public static final double K_CARPET_BLUE = 0.09936;
    }

    public static class Billerica {

        public static final double FIRST_AUTO_SHOT_SPEED = 0.60;

    } 
}
