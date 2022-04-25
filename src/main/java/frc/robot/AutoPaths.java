package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public final class AutoPaths{

    private static DifferentialDriveVoltageConstraint voltageConstraint1 = 
        new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
          Constants.DriveTrain.KINEMATICS,
          10);

        private static TrajectoryConfig config1 =
        new TrajectoryConfig(
          Constants.DriveTrain.MAX_VELOCITY,
          Constants.DriveTrain.MAX_ACCELERATION)
          .setKinematics(Constants.DriveTrain.KINEMATICS)
          .addConstraint(voltageConstraint1).setReversed(true);
    
        //Start of 4 ball auto to first ball
        public static final Trajectory TO_FIRST_BALL =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(6.324, 2.4), Rotation2d.fromDegrees(28.5)), //starting position
          List.of(), //nodes for robot to travel to
          new Pose2d(new Translation2d(5.233, 1.8), Rotation2d.fromDegrees(28.5)), //finishing position
          config1);
          // 1.95

        private static DifferentialDriveVoltageConstraint voltageConstraint2 = 
        new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
          Constants.DriveTrain.KINEMATICS,
          10);
    
        private static TrajectoryConfig config2 =
        new TrajectoryConfig(
          Constants.DriveTrain.MAX_VELOCITY,
          Constants.DriveTrain.MAX_ACCELERATION)
          .setKinematics(Constants.DriveTrain.KINEMATICS)
          .addConstraint(voltageConstraint2).setReversed(true);
    
        //Start of 4 ball auto to first ball
        public static final Trajectory TO_HUMAN_LOADER =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(5.233, 1.8), Rotation2d.fromDegrees(28.5)), //starting position
          List.of(), //nodes for robot to travel to
          new Pose2d(new Translation2d(1.08, 0.78), Rotation2d.fromDegrees(45)), //finishing position
          config2);
          //Loader: (1.05, 0.85)
          //(0.98, 0.68)

        private static DifferentialDriveVoltageConstraint voltageConstraint3 = 
        new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
          Constants.DriveTrain.KINEMATICS,
          10);
    
        private static TrajectoryConfig config3 =
        new TrajectoryConfig(
          Constants.DriveTrain.MAX_VELOCITY,
          Constants.DriveTrain.MAX_ACCELERATION)
          .setKinematics(Constants.DriveTrain.KINEMATICS)
          .addConstraint(voltageConstraint3).setReversed(false);
    
        //Start of 4 ball auto to first ball
        public static Trajectory LOADER_TO_SHOOT =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(1.08, 0.78), Rotation2d.fromDegrees(45)), //starting position
          List.of(), //nodes for robot to travel to
          new Pose2d(new Translation2d(4.715, 2), Rotation2d.fromDegrees(20)), //finishing position
          config3);

          private static DifferentialDriveVoltageConstraint voltageConstraint4 = 
        new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
          Constants.DriveTrain.KINEMATICS,
          10);
    
        private static TrajectoryConfig config4 =
        new TrajectoryConfig(
          Constants.DriveTrain.MAX_VELOCITY,
          Constants.DriveTrain.MAX_ACCELERATION)
          .setKinematics(Constants.DriveTrain.KINEMATICS)
          .addConstraint(voltageConstraint4).setReversed(true);
    
        //Start of 4 ball auto to first ball
        public static Trajectory TWO_BALL_AUTO =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(2.5, 0), Rotation2d.fromDegrees(0)), //starting position
          List.of(), //nodes for robot to travel to
          new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)), //finishing position
          config4);

          private static DifferentialDriveVoltageConstraint voltageConstraint5 = 
        new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
          Constants.DriveTrain.KINEMATICS,
          10);

        private static TrajectoryConfig config5 =
        new TrajectoryConfig(
          Constants.DriveTrain.MAX_VELOCITY,
          Constants.DriveTrain.MAX_ACCELERATION)
          .setKinematics(Constants.DriveTrain.KINEMATICS)
          .addConstraint(voltageConstraint5).setReversed(true);
    
        //Start of 4 ball auto to first ball
        public static final Trajectory HUB_TO_FIRST_BALL =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(7.594, 3.237), Rotation2d.fromDegrees(69)), //starting position
          List.of(), //nodes for robot to travel to
          new Pose2d(new Translation2d(5.233, 1.91), Rotation2d.fromDegrees(15)), //finishing position
          config5);

          private static DifferentialDriveVoltageConstraint voltageConstraint6 = 
        new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
          Constants.DriveTrain.KINEMATICS,
          10);
    
        private static TrajectoryConfig config6 =
        new TrajectoryConfig(
          Constants.DriveTrain.MAX_VELOCITY,
          Constants.DriveTrain.MAX_ACCELERATION)
          .setKinematics(Constants.DriveTrain.KINEMATICS)
          .addConstraint(voltageConstraint6).setReversed(true);
    
        //Start of 4 ball auto to first ball
        public static Trajectory TWO_BALL_FAR_AUTO =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(1.25, 0), Rotation2d.fromDegrees(0)), //starting position
          List.of(), //nodes for robot to travel to
          new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)), //finishing position
          config6);
}