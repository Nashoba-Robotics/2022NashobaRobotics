package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TestAutoAimCommand;
import frc.robot.commands.ActuateIntakeCommand;
import frc.robot.commands.AutoAimMotionMagicCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.AutoStopIntakeCommand;
import frc.robot.commands.UnAimCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class ApplePi4BallAutoCommand extends SequentialCommandGroup{
    //Starts at approximately 200 degrees
    public ApplePi4BallAutoCommand(){
        addCommands(
            new ActuateIntakeCommand(true), //Start the intake, and go for first ball
            new RunIntakeCommand(),
            new PathFollowCommand("paths/AppleTauOver2ToFirstBall.wpilib.json", 200),
            new AutoStopIntakeCommand(),

            new AutoAimMotionMagicCommand(true),    //Aim, shoot, and unaim to reorient
            new AutoShootCommand(Angle.SIXTY),
            new UnAimCommand(),

            new RunIntakeCommand(), //Go grab the third ball near the human player
            new PathFollowCommand("paths/AppleTauOver2ToSecondBall.wpilib.json", 200),
            new AutoStopIntakeCommand(),

            new PathFollowCommand("paths/AppleTauOver2BackToShoot.wpilib.json", 200),   //Go back to shoot
            new AutoAimMotionMagicCommand(true),
            new AutoShootCommand(Angle.SIXTY)
        );
    }
}
