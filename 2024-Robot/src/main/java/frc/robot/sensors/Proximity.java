package frc.robot.sensors;

//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Proximity {
    public static DigitalInput shooterProximity = new DigitalInput(Constants.CANInfo.SHOOTER_PROXIMITY_PORT);
    public static DigitalInput feederProximity = new DigitalInput(Constants.CANInfo.FEEDER_PROXIMITY_PORT);
    public static DigitalInput carriageProximity = new DigitalInput(Constants.CANInfo.CARRIAGE_PROXIMITY_PORT);

    public Proximity(){}

    public static boolean getShooterProximity(){
        return shooterProximity.get();
    }

    public static boolean getCarriageProximity(){
        return carriageProximity.get();
    }
     
    public static boolean getFeederProximity(){
        return feederProximity.get();
    }

    public void periodic(){
        SmartDashboard.putBoolean("Shooter Proximity", getShooterProximity());
        SmartDashboard.putBoolean("Carriage Proximity", getCarriageProximity());
        SmartDashboard.putBoolean("Feeder Proximity", getFeederProximity());
        // Logger.recordOutput("Shooter Proximity", getShooterProximity());
        // Logger.recordOutput("Carriage Proximity", getCarriageProximity());
        // Logger.recordOutput("Feeder Proximity", getFeederProximity());
    }
}