package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DeployClimberCommadGroup extends SequentialCommandGroup{
    public DeployClimberCommadGroup(){
        addCommands(
            new ClimbPrepCommand(),
            new DeployClimberCommand()
        );
    }
}
