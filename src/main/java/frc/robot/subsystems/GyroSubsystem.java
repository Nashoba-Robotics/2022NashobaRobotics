package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase{
    public PigeonIMU pigeon;
    private double[] xyz;
    private double[] speedxyz;
    private short[] acceleration;
    //private TalonSRX talonSRX;

    public GyroSubsystem(){
        //talonSRX = new TalonSRX(4);
        pigeon = new PigeonIMU(0);
        xyz = new double[3];
        speedxyz = new double[3];
        acceleration = new short[3];
        pigeon.setYaw(0);   //In degrees
    }

    private static GyroSubsystem singleton;
    public static GyroSubsystem getInstance(){
        if(singleton == null) singleton = new GyroSubsystem();
        //Yi is a uhcuhuhajnbiciwinr
        return singleton;
    }

    public void zeroHeading(){
        pigeon.setYaw(0);
    }

    public void setAngle(double angle){
        pigeon.setYaw(angle);
    }

    public double getAbsoluteAngle(){
        return getYawPitchRole()[0] % 360;
    }

    public double[] getYawPitchRole(){
        double[] YPR = new double[3];
        pigeon.getYawPitchRoll(YPR);
        return YPR;
    }

    public double[] getArray(){
        pigeon.getAccumGyro(xyz);
        return xyz;
    }

    public double getX(){
        pigeon.getAccumGyro(xyz);
        return xyz[0];
    }

    public double getY(){
        pigeon.getAccumGyro(xyz);
        return xyz[1];
    }

    public double getZ(){
        pigeon.getAccumGyro(xyz);
        return xyz[2];
    }

    public double[] getSpeedArray(){
        pigeon.getRawGyro(speedxyz);
        return speedxyz;
    }

    public double getSpeedX(){
        pigeon.getRawGyro(speedxyz);
        return speedxyz[0];
    }

    public double getSpeedY(){
        pigeon.getRawGyro(speedxyz);
        return speedxyz[1];
    }

    public double getSpeedZ(){
        pigeon.getRawGyro(speedxyz);
        return speedxyz[2];
    }

    public double getAccelX(){
        pigeon.getBiasedAccelerometer(acceleration);
        return acceleration[0];
    }

    public double getAccelY(){
        pigeon.getBiasedAccelerometer(acceleration);
        return acceleration[1];
    }

    public double getAccelZ(){
        pigeon.getBiasedAccelerometer(acceleration);
        return acceleration[2];
    }
}