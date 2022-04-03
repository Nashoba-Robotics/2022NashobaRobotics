package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LedSubsystem;

public class TemporaryReleaseCommand extends ParallelCommandGroup{
    public TemporaryReleaseCommand(){
        addCommands(
            new ReleaseClimberCommand(),
            new ReleasePusherCommand()
        );
    }

}
