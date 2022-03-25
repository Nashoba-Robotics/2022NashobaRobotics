package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
}
