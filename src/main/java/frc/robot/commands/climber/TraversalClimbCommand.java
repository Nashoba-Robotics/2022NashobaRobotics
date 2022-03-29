package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.LedSubsystem;

public class TraversalClimbCommand extends SequentialCommandGroup{
    public TraversalClimbCommand(){
        addCommands(
            new ParallelCommandGroup(   //Climbs onto Mid Bar and raises pushers above the High Bar
                new RetractClimberCommand(),
                new DeployPusher()
            )
            //new PushCommand()   //Pushes Robot so the hooks latch onto the traversal bar
            // new ParallelCommandGroup(    //Releases the robot from the mid and high bar
            //     new ReleaseClimberCommand(),
            //     new ReleasePusherCommand()
            // )
        );
    }

    @Override
    public void initialize() {
        Robot.enableBallLeds = false;
        LedSubsystem.getInstance().twinkle(0, 0, 255);
    }
}
