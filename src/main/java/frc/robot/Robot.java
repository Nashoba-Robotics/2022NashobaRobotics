/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.IntakePracticeCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.autoroutines.TestPathCommand;
import frc.robot.commands.autoroutines.ThreeBallAuto;
import frc.robot.commands.autoroutines.TwoBallAuto;
import frc.robot.commands.autoroutines.TwoBallAuto_Far;
import frc.robot.commands.climber.StopClimbCommand;
import frc.robot.commands.climber.ZeroClimberSensorsCommand;
import frc.robot.commands.climber.ZeroPusherCommand;
import frc.robot.commands.intakeshoot.CannonAngleCommand;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSolenoidSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private PowerDistribution pdh;
    Compressor compressor;
    PneumaticHub ph = new PneumaticHub();
    SendableChooser<Command> autoChooser;

    public static boolean enableBallLeds = true;

    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        // Instantiate our RobotContainer.    This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        new RobotContainer();
        
        pdh = new PowerDistribution();
        pdh.setSwitchableChannel(true);

        LimelightSubsystem.getInstance().setShooterLed(1);

        ph.enableCompressorAnalog(100, 120);
        //ph.disableCompressor();
        CommandScheduler.getInstance().setDefaultCommand(DriveSubsystem.getInstance(), new JoystickDriveCommand());

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Two Ball Auto", new TwoBallAuto());
        autoChooser.addOption("Three Ball Auto", new ThreeBallAuto());
        autoChooser.addOption("Test Auto", new TestPathCommand());
        autoChooser.addOption("Two Ball Far", new TwoBallAuto_Far());
        SmartDashboard.putData("Auto", autoChooser); 

        CommandScheduler.getInstance().schedule(new StopClimbCommand());     
    }

    // long lastMillis = System.currentTimeMillis();

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // long millis = System.currentTimeMillis();
        // if(millis / 500 > lastMillis / 500) {
        //     SmartDashboard.putNumber("loop time", millis - lastMillis);
        // }
        // lastMillis = millis;
    }

    @Override
    public void disabledInit() {    //Ensures that everything is stopped and is in an "off" state
        DriveSubsystem.getInstance().setDriveMode(DriveMode.VELOCITY);
        DriveSubsystem.getInstance().setSpeed(0, 0);
        IntakeSubsystem.getInstance().stop();
        // IntakeSolenoidSubsystem.getInstance().retract();    //Undeploys the intake when the robot is disabled
        CannonSubsystem.getInstance().setAngle(Angle.EIGHTY);
        DriveSubsystem.getInstance().changeNeutralMode(NeutralMode.Coast);    //Sets the robot into "coast" mode after robot is diabled -> Easier to move
        LimelightSubsystem.getInstance().setShooterLed(1);
        ClimberSubsystem.getInstance().stop();
        CommandScheduler.getInstance().schedule(new StopClimbCommand());
        //Cancels everything that's running
        CommandScheduler.getInstance().cancelAll();

        if(DriverStation.isFMSAttached()) {
            LedSubsystem.getInstance().rgbfade();
        } else {
            LedSubsystem.getInstance().setColor(0,0,0);
        }
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        LedSubsystem.getInstance().rainbow();
        LimelightSubsystem.getInstance().setShooterLed(3);
        DriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        DriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        DriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        autoChooser.getSelected().schedule();
    }

    @Override
    public void autonomousPeriodic() {        
    }
        
    @Override
    public void teleopInit() {
        LiveWindow.disableAllTelemetry();
        CommandScheduler.getInstance().cancelAll();

        LimelightSubsystem.getInstance().setShooterLed(3);

        DriveSubsystem.getInstance().resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        CommandScheduler.getInstance().schedule(new StopClimbCommand());
        // CommandScheduler.getInstance().schedule(new ZeroClimberSensorsCommand());
        // CommandScheduler.getInstance().schedule(new ZeroPusherCommand());

        CommandScheduler.getInstance().schedule(new CannonAngleCommand(Angle.EIGHTY));

        //Zeroes the climbers when teleop starts
        //CommandScheduler.getInstance().schedule(new ZeroClimberCommand());

        enableBallLeds = true;
    }

    @Override
    public void teleopPeriodic() {
        if(enableBallLeds) {
            if(RobotContainer.getSensor1() && RobotContainer.getSensor2()){
                LedSubsystem.getInstance().setColor(255, 0, 0);
            } else if(RobotContainer.getSensor1() || RobotContainer.getSensor2()){
                LedSubsystem.getInstance().setColor(0, 0, 255);
            } else{
                LedSubsystem.getInstance().setColor(255, 255, 255);
            }
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().setDefaultCommand(DriveSubsystem.getInstance(), new JoystickDriveCommand());
        CommandScheduler.getInstance().schedule(new IntakePracticeCommand());
    }

    @Override
    public void testPeriodic() {
    }
}