package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.intakeshoot.ActuateIntakeCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class ThreeBallAuto_Far extends SequentialCommandGroup{
    public ThreeBallAuto_Far(){
        addCommands(
            new AutoShootCommand(Angle.SIXTY),
            new ActuateIntakeCommand(true),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                new RunIntakeCommand().until(() -> {
                    return RobotContainer.getSensor1() && RobotContainer.getSensor2();
                }
                ).withTimeout(5),
                new SequentialCommandGroup(
                    new PathFollowCommand("paths/ThreeBallToBall_Far.wpilib.json"),
                    new PathFollowCommand("paths/ThreeBallToSecondBall_Right.wpilib.json")  
                )
            ),
            new ActuateIntakeCommand(false),
            new PathFollowCommand("paths/ThreeBallToShoot_Right.wpilib.json"),
            new AutoAimCommand(),
            new AutoShootCommand(Angle.SIXTY)
        );
    }
}
