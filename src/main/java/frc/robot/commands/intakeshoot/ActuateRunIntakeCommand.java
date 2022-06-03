package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ActuateRunIntakeCommand extends SequentialCommandGroup{
    
    public ActuateRunIntakeCommand() {
        addCommands(
            new ActuateIntakeCommand(true),
            new RunIntakeCommand()
        );
    }

}
