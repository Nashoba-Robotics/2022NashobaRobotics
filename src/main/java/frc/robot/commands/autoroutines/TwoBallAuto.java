package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.intakeshoot.CannonAngleCommand;
import frc.robot.commands.intakeshoot.ActuateIntakeCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(){
        addRequirements(DriveSubsystem.getInstance());
        
        addCommands(
            new AutoShootCommand(Angle.EIGHTY),
            new ActuateIntakeCommand(true),
            new ParallelCommandGroup(
                new RunIntakeCommand().until(RobotContainer::getSensor2).withTimeout(3),
                DriveSubsystem.getInstance().getAutonomousCommand("paths/TwoBallToBall.wpilib.json")
            ),
            new ActuateIntakeCommand(false),
            DriveSubsystem.getInstance().getAutonomousCommand("paths/TwoBallFromBallToShoot.wpilib.json"),
            new AutoAimCommand(),
            new AutoShootCommand(Angle.SIXTY)
        );
    }
}
