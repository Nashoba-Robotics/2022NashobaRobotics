package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TraversalClimbCommand extends SequentialCommandGroup{
    public TraversalClimbCommand(){
        addCommands(
            new ParallelCommandGroup(
                new RetractClimberCommand(),
                new DeployPusher()
            )
        );
    }
}
