package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.climber.DeployPusher;
import frc.robot.commands.climber.RetractClimberCommand;

public class ParallelTestCommand extends ParallelCommandGroup{
    public ParallelTestCommand(){
        addCommands(
            new RetractClimberCommand(),
            new DeployPusher()
        );
    }
}
