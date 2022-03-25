package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.SetStartAngleCommand;
import frc.robot.commands.intakeshoot.CannonAngleCommand;
import frc.robot.commands.intakeshoot.ActuateIntakeCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(){
        addRequirements(DriveSubsystem.getInstance());
        
        addCommands(
            new SetStartAngleCommand(Constants.DriveTrain.CLOSE_LEFT_START_ANGLE),
            // new AutoShootCommand(Angle.EIGHTY),
            // new ActuateIntakeCommand(true),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                //new RunIntakeCommand().until(RobotContainer::getSensor2).withTimeout(3),
                new PathFollowCommand("paths/TwoBallToBall.wpilib.json")
            ),
            // new ActuateIntakeCommand(false),
            new PathFollowCommand("paths/TwoBallFromBallToShoot.wpilib.json")
            //new AutoAimCommand(),
            //new AutoShootCommand(Angle.SIXTY)
        );
    }
}
