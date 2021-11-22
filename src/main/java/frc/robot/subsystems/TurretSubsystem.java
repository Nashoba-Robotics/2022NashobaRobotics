package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    private static TurretSubsystem instance;
    private TalonSRX turret;

    private TurretSubsystem() {
        turret = new TalonSRX(Constants.TURRET_PORT);

        turret.configFactoryDefault();
    }

    public static TurretSubsystem getInstance() {
        if(instance == null) {
            instance = new TurretSubsystem();
        }
        return instance;
    }

    public void setVelocity(double velocity) {
        turret.set(ControlMode.PercentOutput, velocity);
    }

    
}