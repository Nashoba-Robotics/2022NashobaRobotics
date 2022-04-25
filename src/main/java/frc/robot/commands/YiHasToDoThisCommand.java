package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.Units;
import frc.robot.subsystems.DriveSubsystem;

public class YiHasToDoThisCommand extends CommandBase {
    public enum Direction {
        FORWARD, BACK, LEFT, RIGHT
    }

    private Direction direction;
    private Alliance alliance;

    private double startR;
    private double startL;

    private static double forwardMeters;

    public YiHasToDoThisCommand(Direction d, Alliance a) {
        direction = d;
        alliance = a;
    }

    @Override
    public void initialize() {
        startL = Units.NU2Meters(DriveSubsystem.getInstance().getPositionLeft());
        startR = Units.NU2Meters(DriveSubsystem.getInstance().getPositionLeft());
    }

    @Override
    public void end(boolean interrupted) {
        double endL = Units.NU2Meters(DriveSubsystem.getInstance().getPositionLeft());
        double endR = Units.NU2Meters(DriveSubsystem.getInstance().getPositionLeft());
        double l = endL - startL;
        double r = endR - startR;
        String name = "FIELD TEST " + direction.toString() + " " + alliance.toString() + " ";
        if(direction == Direction.FORWARD){
            double radius = (((l + r) / 2) / 3.225) * Constants.DriveTrain.WHEEL_RADIUS;
            forwardMeters = (l + r) / 2;
            SmartDashboard.putNumber(name + "new radius", radius);
            System.out.println(name + "new radius: " + radius);
        }
        if(direction == Direction.BACK){
            try{
                double carpet = Math.abs(forwardMeters / ((l + r) / 2)) - 1;
                SmartDashboard.putNumber(name + "K_Carpet", carpet);
                System.out.println(name + "K_Carpte: " + carpet);
            } catch(Exception e) {
                System.out.println("no change in motor values");
            }
        }
        SmartDashboard.putNumber(name + "LEFT", l);
        SmartDashboard.putNumber(name + "RIGHT", r);
        System.out.println(name + "LEFT: " + l);
        System.out.println(name + "RIGHT: " + r);
    }
    
    
}
