package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class TraversalClimbCommand extends ParallelCommandGroup{
    public TraversalClimbCommand(){

        addCommands(
            new RetractClimberCommand(),
            new DeployPusher()
        );

        // addCommands(
        //      //Climbs onto Mid Bar and raises pushers above the High Bar
        //     new RetractClimberCommand(),
        //     new DeployPusher()
            
        //     //new PushCommand()   //Pushes Robot so the hooks latch onto the traversal bar
        //     // new ParallelCommandGroup(    //Releases the robot from the mid and high bar
        //     //     new ReleaseClimberCommand(),
        //     //     new ReleasePusherCommand()
        //     // )
        // );
    }
}
