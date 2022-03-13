package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoRoutineCommandGroup extends SequentialCommandGroup{
    public AutoRoutineCommandGroup(){
        addRequirements(DriveSubsystem.getInstance());

        addCommands(
            DriveSubsystem.getInstance().getAutonomousCommand("")
        );
    }
}
