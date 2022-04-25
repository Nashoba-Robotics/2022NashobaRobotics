package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TestAutoAimCommand;
import frc.robot.commands.UnAimCommand;
import frc.robot.commands.YiHasToDoThisCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.LedTestCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.climber.ManualClimberCommand;
import frc.robot.commands.climber.DeployClimberCommadGroup;
import frc.robot.commands.climber.DeployPusher;
import frc.robot.commands.climber.ManualPusherCommand;
import frc.robot.commands.climber.PushCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.climber.StopClimbCommand;
import frc.robot.commands.climber.TemporaryReleaseCommand;
import frc.robot.commands.climber.TraversalClimbCommand;
import frc.robot.commands.climber.ZeroClimberSensorsCommand;
import frc.robot.commands.climber.ZeroPusherCommand;
import frc.robot.commands.intakeshoot.CannonAngleCommand;
import frc.robot.commands.intakeshoot.ActuateIntakeCommand;
import frc.robot.commands.intakeshoot.AimShootCG;
import frc.robot.commands.intakeshoot.EjectBackCommand;
import frc.robot.commands.intakeshoot.EjectFrontCommand;
import frc.robot.commands.intakeshoot.ShootCommand;
import frc.robot.commands.intakeshoot.PukeCommand;
import frc.robot.commands.intakeshoot.RunIntakeCommand;
import frc.robot.commands.intakeshoot.ShootCommand;
import frc.robot.commands.intakeshoot.StopIntakeCommand;
import frc.robot.commands.intakeshoot.StopShooterCommand;
import frc.robot.commands.intakeshoot.ToggleAutoAimCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.CannonSubsystem.Angle;
import frc.robot.commands.AutoAimMotionMagicCommand;
import frc.robot.commands.CannonTestCommand;
import frc.robot.commands.ConfigDriveCommand;

public class RobotContainer {

    public static JoystickDriveCommand joystickDriveCommand = new JoystickDriveCommand();

    //Initializes all buttons used for commands
    //Takes in a joystick and the button poer
    //IMPORTANT!!: Joystick Buttons are 1 indexed (They start at 1 instead of 0)
    Trigger deployIntakeSwitch = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.DEPLOY_INTAKE);

    Trigger runIntakeButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.RUN_INTAKE).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    Trigger stopIntakeButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.STOP_INTAKE).debounce(Constants.Buttons.DEBOUNCE_VALUE);
                                                                                                                                                                                                                                                                    
    Trigger ejectFrontButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.EJECT_FRONT).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    Trigger ejectBackButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.EJECT_BACK).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    Trigger pukeButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.PUKE).debounce(Constants.Buttons.DEBOUNCE_VALUE);

    Trigger shootButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.SHOOT).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    Trigger runShooterButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.RUN_SHOOTER).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    Trigger stopShooterButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.STOP_SHOOTER);
    public static Trigger shooterAngleSwitch = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.SHOOTER_ANGLE).debounce(Constants.Buttons.DEBOUNCE_VALUE);

    Trigger fixedClimbDeployButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.FIXED_CLIMB_DEPLOY).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    Trigger fixedClimbButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.FIXED_CLIMB).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    Trigger traversalClimbButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.TRAVERSAL_CLIMB).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    Trigger fixedClimberReleaseButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Buttons.FIXED_CLIMB_RELEASE).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    
    Trigger enableManualPushButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.ENABLE_MANUAL_PUSH);
    Trigger enableManualClimbButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), Constants.Buttons.ENABLE_MANUAL_CLIMB);

    Trigger colorRejectionSwitch = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), Constants.Intake.COLOR_REJECTION_SWITCH_PORT);

    // Trigger rotatingClimbDeployButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 6).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    // Trigger rotatingClimbUndeployButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 5).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    Trigger rotatingClimbeGrabButton = new JoystickButton(JoystickSubsystem.getInstance().getRightOperatorJoystick(), 4).debounce(Constants.Buttons.DEBOUNCE_VALUE);
    //Trigger rotatingClimberReleaseButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftOperatorJoystick(), 5).debounce(Constants.Buttons.DEBOUNCE_VALUE);

    // Trigger autoAimButton = new JoystickButton(JoystickSubsystem.getInstance().getRightJoystick(), Constants.Buttons.AUTO_AIM);

    // JoystickButton incrementShooterSpeed = new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), /* TODO */);
    // JoystickButton decrementShooterSpeed = new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), /* TODO */);

    Trigger autoAimButton = new JoystickButton(JoystickSubsystem.getInstance().getRightJoystick(), Constants.Buttons.AUTO_AIM);
    Trigger onAim = new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 13);
    Trigger offAim = new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 14);
    Trigger emergencyConfigButton = new JoystickButton(JoystickSubsystem.getInstance().getLeftJoystick(), 11);

    static DigitalInput ballSensor1 = new DigitalInput(Constants.Intake.DIO_SENSOR_1);
    static DigitalInput ballSensor2 = new DigitalInput(Constants.Intake.DIO_SENSOR_2);

    public static boolean getSensor1() {
        return !ballSensor1.get();
    }
    public static boolean getSensor2() {
        return !ballSensor2.get();
    }

    public RobotContainer() {
        configureButtonBindings();
        // SmartDashboard.putData(new ZeroClimberSensorsCommand());
        // SmartDashboard.putData(new ZeroPusherCommand());
        SmartDashboard.putData(new StopClimbCommand());
        // SmartDashboard.putData(new CannonTestCommand());
        // SmartDashboard.putData(new LimelightCommand());
        // SmartDashboard.putData(new LedTestCommand());

        // SmartDashboard.putData(new RetractClimberCommand());
        // SmartDashboard.putData(new DeployPusher());
        // SmartDashboard.putData(new TraversalClimbCommand());

        // SmartDashboard.putData(new AimShootCG());

        // SmartDashboard.putData(new TestAutoAimCommand());
        // SmartDashboard.putData(new AutoAimMotionMagicCommand(false));
        // SmartDashboard.putData(new UnAimCommand());

        // SmartDashboard.putData("YHTDT Forward Red", new YiHasToDoThisCommand(YiHasToDoThisCommand.Direction.FORWARD, Alliance.Red));
        // SmartDashboard.putData("YHTDT Back Red", new YiHasToDoThisCommand(YiHasToDoThisCommand.Direction.BACK, Alliance.Red));
        // SmartDashboard.putData("YHTDT Forward Blue", new YiHasToDoThisCommand(YiHasToDoThisCommand.Direction.FORWARD, Alliance.Blue));
        // SmartDashboard.putData("YHTDT Back Blue", new YiHasToDoThisCommand(YiHasToDoThisCommand.Direction.BACK, Alliance.Blue));
    }

    private void configureButtonBindings() {
        deployIntakeSwitch.whenInactive(new ActuateIntakeCommand(true));
        deployIntakeSwitch.whenActive(new ActuateIntakeCommand(false));

        // emergencyConfigButton.whenActive(() -> {DriveSubsystem.getInstance().emergencyConfig();});
        emergencyConfigButton.whenActive(new ConfigDriveCommand());

        // incrementShooterSpeed.whenActive(() -> {CannonSubsystem.getInstance().changeSpeedChange(0.005);});
        // decrementShooterSpeed.whenActive(() -> {CannonSubsystem.getInstance().changeSpeedChange(-0.005);});
        runIntakeButton.whenActive(new RunIntakeCommand());
        stopIntakeButton.whenActive(new StopIntakeCommand());

        shooterAngleSwitch.whenActive(new CannonAngleCommand(Angle.EIGHTY));
        shooterAngleSwitch.whenInactive(new CannonAngleCommand(Angle.SIXTY));

        ejectFrontButton.whenActive(new EjectFrontCommand());
        ejectBackButton.whenActive(new EjectBackCommand());
        pukeButton.whenActive(new PukeCommand());

        //ShootCommand shootCommand = new ShootCommand();
        runShooterButton.whenActive(new ShootCommand(false));
        shootButton.whenActive(new AimShootCG());
        stopShooterButton.whenActive(new StopShooterCommand());

        fixedClimbDeployButton.whenActive(new DeployClimberCommadGroup());
        fixedClimbButton.whenActive(new TraversalClimbCommand());
        traversalClimbButton.whenActive(new PushCommand());

        enableManualPushButton.whenActive(new ManualPusherCommand());
        enableManualClimbButton.whenActive(new ManualClimberCommand());

        

        
        // colorRejectionSwitch.whenActive(new SetColorRejectionCommand(true));
        // colorRejectionSwitch.whenInactive(new SetColorRejectionCommand(false));
        // fixedClimbeGrabButton.whenActive(new StopCommand());    //TODO: Change to actual Grab Command
        //fixedClimberReleaseButton.whenActive(new StopIntakeCommand());

        // rotatingClimbDeployButton.whenActive(new StopCommand());    //TODO: Change to actual Rotating Deploy Command
        // rotatingClimbUndeployButton.whenActive(new StopCommand());    //TODO: Change to actual Rotating Undeploy Command
        // rotatingClimbeGrabButton.whenActive(new StopCommand());     //TODO: Change to actual Rotating Grab Command
        // rotatingClimberReleaseButton.whenActive();
        rotatingClimbeGrabButton.whenActive(new TemporaryReleaseCommand());     //TODO: Change to actual Rotating Grab Command
        //rotatingClimberReleaseButton.whenActive(new TemporaryReleaseCommand());

        autoAimButton.toggleWhenActive(new AutoAimMotionMagicCommand(false));

        onAim.whenActive(new ToggleAutoAimCommand(true));
        offAim.whenActive(new ToggleAutoAimCommand(false));
    }

}
