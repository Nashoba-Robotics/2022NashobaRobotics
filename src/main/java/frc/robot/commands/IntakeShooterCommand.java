package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeShooterCommand extends CommandBase {
    State state;

    public IntakeShooterCommand(){
        addRequirements(IntakeSubsystem.getInstance());
        addRequirements(CannonSubsystem.getInstance());
        SmartDashboard.putNumber("Shoot", 0);
        SmartDashboard.putNumber("Puke", 0);
        SmartDashboard.putNumber("Puke All", 0);
        SmartDashboard.putNumber("Stop Emitting", 0);
    }

    @Override
    public void initialize(){
        state = State.NO_BALLS;
    }

    @Override
    public void execute(){
        state = state.updateSensors(
            IntakeSubsystem.getInstance().getSensor1(), 
            IntakeSubsystem.getInstance().getSensor2()
        );
        if(SmartDashboard.getNumber("Shoot", 0) != 0) {
            SmartDashboard.putNumber("Shoot", 0);
            state = state.shoot();
        }
        if(SmartDashboard.getNumber("Puke", 0) != 0) {
            SmartDashboard.putNumber("Puke", 0);
            state = state.puke();
        }
        if(SmartDashboard.getNumber("Puke All", 0) != 0) {
            SmartDashboard.putNumber("Puke All", 0);
            state = state.puke();
        }
        if(SmartDashboard.getNumber("Stop Emitting", 0) != 0) {
            SmartDashboard.putNumber("Stop Emitting", 0);
            state = state.donePuking().doneShooting();
        }
        switch(state) {
            case SHOOT:
            case SHOOT_BOTH:
                CannonSubsystem.getInstance().set(0.5);
            default:
                CannonSubsystem.getInstance().set(0);
        }
        switch(state) {
            case NO_BALLS:
                IntakeSubsystem.getInstance()
                    .setIntake(0.5)
                    .setGrabber(0.5)
                    .setLoader(0.2);
                break;
            case ONE_BALL:
                IntakeSubsystem.getInstance()
                    .setIntake(0.5)
                    .setGrabber(0.5)
                    .setLoader(0);
                break;
            case TWO_BALLS:
                IntakeSubsystem.getInstance()
                    .setIntake(0)
                    .setGrabber(0)
                    .setLoader(0);
                break;
            case SHOOT:
            case SHOOT_BOTH:
                IntakeSubsystem.getInstance()
                    .setIntake(0)
                    .setGrabber(0)
                    .setLoader(0.2);
                break;
            case PUKE_FIRST:
            case PUKE_SECOND:
            case PUKE_BOTH:
                IntakeSubsystem.getInstance()
                    .setIntake(-0.4)
                    .setGrabber(-0.4)
                    .setLoader(0);
        }
    }

    @Override
    public void end(boolean interrupted){
        IntakeSubsystem.getInstance().stop();
        CannonSubsystem.getInstance().set(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public static enum State {
        NO_BALLS,
        ONE_BALL,
        TWO_BALLS,
        SHOOT,
        SHOOT_BOTH,
        PUKE_FIRST,
        PUKE_SECOND,
        PUKE_BOTH;

        public State shoot() {
            switch(this) {
                case ONE_BALL:
                    return SHOOT;
                case TWO_BALLS:
                    return SHOOT_BOTH;
                default:
                    return this;
            }
        } 

        public State doneShooting() {
            switch(this) {
                case SHOOT:
                case SHOOT_BOTH:
                    return NO_BALLS;
                default:
                    return this;
            }
        }

        public State puke() {
            switch(this) {
                case ONE_BALL:
                    return PUKE_FIRST;
                case TWO_BALLS:
                    return PUKE_SECOND;
                default:
                    return this;
            }
        }

        public State pukeAll() {
            switch(this) {
                case ONE_BALL:
                    return PUKE_FIRST;
                case TWO_BALLS:
                    return PUKE_BOTH;
                default:
                    return this;
            }
        }

        public State donePuking() {
            switch(this) {
                case PUKE_FIRST:
                case PUKE_BOTH:
                    return NO_BALLS;
                case PUKE_SECOND:
                    return ONE_BALL;
                default:
                    return this;
            }
        }

        public State updateSensors(boolean s1, boolean s2) {
            switch(this) {
                case NO_BALLS:
                    return s2 ? ONE_BALL : NO_BALLS;
                case ONE_BALL:
                    return s1 ? TWO_BALLS : ONE_BALL;
                default:
                    return this;
            }
        }
    }
}