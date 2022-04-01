package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoShootCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.intakeshoot.CannonAngleCommand;
import frc.robot.commands.intakeshoot.ActuateIntakeCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class ThreeBallAuto extends SequentialCommandGroup{
    public ThreeBallAuto(){
        addCommands(
            // new SetStartAngleCommand(Constants.DriveTrain.CLOSE_LEFT_START_ANGLE),
            // new AutoShootCommand(Angle.EIGHTY),
            // new ActuateIntakeCommand(true),
            // new WaitCommand(0.25),
            // new ParallelCommandGroup(
            //     new RunIntakeCommand().until(() -> {
            //         return RobotContainer.getSensor1() && RobotContainer.getSensor2();
            //     }
            //     ).withTimeout(5),
            //     new SequentialCommandGroup(
            //         new PathFollowCommand("paths/ThreeBallToBall.wpilib.json"),
            //         new PathFollowCommand("paths/ThreeBallToSecondBall.wpilib.json")  
            //     )
            // ),
            // new ActuateIntakeCommand(false),
            // new PathFollowCommand("paths/ThreeBallToShoot.wpilib.json"),
            // new AutoAimCommand(),
            // new AutoShootCommand(Angle.SIXTY)
        );
    }
}
