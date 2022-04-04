package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class TemporaryReleaseCommand extends ParallelCommandGroup{
    public TemporaryReleaseCommand(){
        addCommands(
            new ReleaseClimberCommand(),
            new ReleasePusherCommand()
        );
    }

}
