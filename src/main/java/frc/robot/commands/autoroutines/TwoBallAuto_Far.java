package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.intakeshoot.CannonAngleCommand;
import frc.robot.commands.intakeshoot.ActuateIntakeCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class TwoBallAuto_Far extends SequentialCommandGroup {
    public TwoBallAuto_Far(){
        addRequirements(DriveSubsystem.getInstance());
        addCommands(
            // new AutoShootCommand(Angle.EIGHTY),
            // new ActuateIntakeCommand(true),
            // new WaitCommand(0.5),
            // new ParallelCommandGroup(
            //     new RunIntakeCommand().until(RobotContainer::getSensor2).withTimeout(3),
            //     new PathFollowCommand("paths/ToBall_LeftFar.wpilib.json")
            // ),
            // new ActuateIntakeCommand(false),
            // new PathFollowCommand("paths/TwoBallFromBallToShoot.wpilib.json"),
            // new AutoAimCommand(),
            // new AutoShootCommand(Angle.SIXTY)
        );
    }
}

// to be or not to be that is the question whether tis nobler in the mind to suffer the slings of outrageous fortune or cast uhh uhh to die to sleep to sleep perchance to sleep to die to dream perchance to die to dream perchance