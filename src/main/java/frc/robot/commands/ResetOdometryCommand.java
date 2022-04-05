package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometryCommand extends CommandBase{
    Trajectory trajectory;
    double angOfResistance;

    Timer timer;

    public ResetOdometryCommand(Trajectory trajectory, double angOfResistance){
        this.trajectory = trajectory;
        this.angOfResistance = angOfResistance;
        timer = new Timer();
        timer.start();
    }

    @Override
    public void initialize() {
        timer.reset();
    }
    
    @Override
    public void execute() {
        DriveSubsystem.getInstance().resetOdometry(trajectory.getInitialPose(), angOfResistance);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 0.1;
    }
}
