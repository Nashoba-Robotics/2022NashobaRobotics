package frc.robot.commands.climber;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TemporaryReleaseCommand extends ParallelCommandGroup{
    public TemporaryReleaseCommand(){
        addCommands(
            new ReleaseClimberCommand(),
            new ReleasePusherCommand()
        );
    }
}
