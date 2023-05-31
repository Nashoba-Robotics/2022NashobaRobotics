package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActuateIntakeCommand;
import frc.robot.commands.AutoAimMotionMagicCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.UnAimCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoPaths;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.intakeshoot.CannonAngleCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.commands.intakeshoot.ToggleAutoAimCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class ThreeBallAuto extends SequentialCommandGroup{
    public ThreeBallAuto(){
        addCommands(
            new ActuateIntakeCommand(true),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                new RunIntakeCommand().until(() -> {
                    return RobotContainer.getSensor1() && RobotContainer.getSensor2();
                }
                ).withTimeout(5),
                new PathFollowCommand(AutoPaths.TO_FIRST_BALL)
            ),
            new ToggleAutoAimCommand(true),
            new AutoAimMotionMagicCommand(false),
            new AutoShootCommand(Angle.SIXTY),
            new UnAimCommand(),
            new ParallelCommandGroup(
                new RunIntakeCommand().until(() -> {
                    return RobotContainer.getSensor2();
                }
                ).withTimeout(5),
                new PathFollowCommand(AutoPaths.TO_HUMAN_LOADER)
            ),
            new ActuateIntakeCommand(false),
            new PathFollowCommand(AutoPaths.LOADER_TO_SHOOT),
            new AutoAimMotionMagicCommand(false),
            new AutoShootCommand(Angle.SIXTY)
        );
    }
}
