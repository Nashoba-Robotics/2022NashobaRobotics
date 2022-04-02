package frc.robot.commands.intakeshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonSubsystem;

public class ShootSpeedCommand extends CommandBase{

    private double shootSpeed;
    private int finishTimeMilllis;

    private long startMillis;

    public ShootSpeedCommand(double shootSpeed, int finishTimeMillis){
        this.shootSpeed = shootSpeed;
        this.finishTimeMilllis = finishTimeMilllis;
        startMillis = System.currentTimeMillis();

        addRequirements(CannonSubsystem.getInstance());

    }

    @Override
    public void execute() {
        CannonSubsystem.getInstance().set(shootSpeed);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startMillis >= finishTimeMilllis;
    }

    @Override
    public void end(boolean interrupted) {
        CannonSubsystem.getInstance().set(0);
    }
}
