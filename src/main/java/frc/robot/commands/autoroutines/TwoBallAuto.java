package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoPaths;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake;
import frc.robot.commands.AutoAimMotionMagicCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.intakeshoot.CannonAngleCommand;
import frc.robot.commands.intakeshoot.ActuateIntakeCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.commands.intakeshoot.ToggleAutoAimCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(){
        addRequirements(DriveSubsystem.getInstance());
        
        addCommands(
            new AutoShootCommand(Angle.EIGHTY),
            new ActuateIntakeCommand(true),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new RunIntakeCommand().until(RobotContainer::getSensor2).withTimeout(3),
                new PathFollowCommand(AutoPaths.TWO_BALL_AUTO)
            ),
            new ActuateIntakeCommand(false),
            // new PathFollowCommand("paths/TwoBallFromBallToShoot.wpilib.json"),
            new ToggleAutoAimCommand(true),
            new AutoAimMotionMagicCommand(false),
            new AutoShootCommand(Angle.SIXTY)
            // new AutoAimCommand().withTimeout(2),
            // new AutoShootCommand(Angle.SIXTY)
        );
    }
}
