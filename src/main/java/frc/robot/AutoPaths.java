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
          new Pose2d(new Translation2d(6.684, 2.292), Rotation2d.fromDegrees(15)), //starting position
          List.of(), //nodes for robot to travel to
          new Pose2d(new Translation2d(5.233, 1.91), Rotation2d.fromDegrees(15)), //finishing position
          config1);

          DifferentialDriveVoltageConstraint voltageConstraint2 = 
        new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
          Constants.DriveTrain.KINEMATICS,
          10);
    
        TrajectoryConfig config2 =
        new TrajectoryConfig(
          Constants.DriveTrain.MAX_VELOCITY,
          Constants.DriveTrain.MAX_ACCELERATION)
          .setKinematics(Constants.DriveTrain.KINEMATICS)
          .addConstraint(voltageConstraint2).setReversed(true);
    
        //Start of 4 ball auto to first ball
        Trajectory TO_HUMAN_LOADER =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(5.233, 1.91), Rotation2d.fromDegrees(15)), //starting position
          List.of(), //nodes for robot to travel to
          new Pose2d(new Translation2d(1.54, 1.222), Rotation2d.fromDegrees(12)), //finishing position
          config2);

          DifferentialDriveVoltageConstraint voltageConstraint3 = 
        new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(Constants.DriveTrain.KS, Constants.DriveTrain.KV, Constants.DriveTrain.KA),
          Constants.DriveTrain.KINEMATICS,
          10);
    
        TrajectoryConfig config3 =
        new TrajectoryConfig(
          Constants.DriveTrain.MAX_VELOCITY,
          Constants.DriveTrain.MAX_ACCELERATION)
          .setKinematics(Constants.DriveTrain.KINEMATICS)
          .addConstraint(voltageConstraint3).setReversed(false);
    
        //Start of 4 ball auto to first ball
        Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(1.54, 1.222), Rotation2d.fromDegrees(12)), //starting position
          List.of(), //nodes for robot to travel to
          new Pose2d(new Translation2d(3.748, 1.64), Rotation2d.fromDegrees(30)), //finishing position
          config3);
}