package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoPaths;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAimMotionMagicCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.UnAimCommand;
import frc.robot.commands.intakeshoot.ActuateIntakeCommand;
import frc.robot.commands.intakeshoot.CannonAngleCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.commands.intakeshoot.ToggleAutoAimCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class FourBallAuto extends SequentialCommandGroup{
    public FourBallAuto(){
        addCommands(
            new ActuateIntakeCommand(true),
            new CannonAngleCommand(Angle.SIXTY),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                new RunIntakeCommand().until(() -> {
                    return RobotContainer.getSensor1() && RobotContainer.getSensor2();
                }
                ).withTimeout(5),
                // new PathFollowCommand(AutoPaths.HUB_TO_FIRST_BALL, Constants.FIELD.ANGLE_OF_RESISTANCE)
                new PathFollowCommand(AutoPaths.TO_FIRST_BALL, Constants.FIELD.ANGLE_OF_RESISTANCE)
            ),
            new ToggleAutoAimCommand(true),
            new AutoAimMotionMagicCommand(false),
            new AutoShootCommand(Angle.SIXTY),
            new UnAimCommand(),
            new ParallelCommandGroup(
                new RunIntakeCommand().until(() -> {
                    return RobotContainer.getSensor2() && RobotContainer.getSensor1();
                }
                ).withTimeout(5),
                new PathFollowCommand(AutoPaths.TO_HUMAN_LOADER, Constants.FIELD.ANGLE_OF_RESISTANCE)
            ),
            new ParallelCommandGroup(
                new RunIntakeCommand().withTimeout(2),
                new SequentialCommandGroup(
                    new PathFollowCommand(AutoPaths.LOADER_TO_SHOOT, Constants.FIELD.ANGLE_OF_RESISTANCE),
                    new AutoAimMotionMagicCommand(false)
                )
            ),
            new AutoShootCommand(Angle.SIXTY)
        );
    }
}