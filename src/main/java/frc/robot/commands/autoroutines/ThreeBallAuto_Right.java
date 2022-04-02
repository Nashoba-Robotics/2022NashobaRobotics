package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.intakeshoot.ActuateIntakeCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class ThreeBallAuto_Right extends SequentialCommandGroup{
    public ThreeBallAuto_Right(){
        addCommands(
            // new SetStartAngleCommand(Constants.DriveTrain.CLOSE_RIGHT_START_ANGLE),
            // new AutoShootCommand(Angle.EIGHTY),
            // new ActuateIntakeCommand(true),
            // new WaitCommand(0.25),
            // new ParallelCommandGroup(
            //     new RunIntakeCommand().until(() -> {
            //         return RobotContainer.getSensor1() && RobotContainer.getSensor2();
            //     }
            //     ).withTimeout(5),
            //     new SequentialCommandGroup(
            //         new PathFollowCommand("paths/ThreeBallToBall_Right.wpilib.json"),
            //         new PathFollowCommand("paths/ThreeBallToSecondBall_Right.wpilib.json")  
            //     )
            // ),
            // new ActuateIntakeCommand(false),
            // new PathFollowCommand("paths/ThreeBallToShoot.wpilib_Right.json"),
            // new AutoAimCommand(),
            // new AutoShootCommand(Angle.SIXTY)
        );
    }
}
