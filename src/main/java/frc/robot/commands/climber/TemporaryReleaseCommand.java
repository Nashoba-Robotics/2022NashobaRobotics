package frc.robot.commands.climber;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LedSubsystem;

public class TemporaryReleaseCommand extends ParallelCommandGroup{
    public TemporaryReleaseCommand(){
        addCommands(
            new ReleaseClimberCommand(),
            // new DeployClimberCommad(),
            new ReleasePusherCommand()
        );
    }

    public void initialize() {
        Robot.enableBallLeds = false;
        LedSubsystem.getInstance().twinkle(Constants.Leds.TEMPORARY_RELEASE);
    }
}
