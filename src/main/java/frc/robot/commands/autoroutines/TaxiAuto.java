package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoPaths;
import frc.robot.Constants;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.CannonSubsystem.Angle;

public class TaxiAuto extends SequentialCommandGroup{
    public TaxiAuto(){
        addCommands(
            new AutoShootCommand(Angle.EIGHTY),
            new WaitCommand(5),
            new PathFollowCommand(AutoPaths.TWO_BALL_AUTO)
        );
    }
}
