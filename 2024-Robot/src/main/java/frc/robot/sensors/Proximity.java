package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Proximity {
    public final DigitalInput shooterProximity = new DigitalInput(Constants.CANInfo.SHOOTER_PROXIMITY_PORT);
    public final DigitalInput feederProximity = new DigitalInput(Constants.CANInfo.FEEDER_PROXIMITY_PORT);
    public final DigitalInput carriageProximity = new DigitalInput(Constants.CANInfo.CARRIAGE_PROXIMITY_PORT);

    public Proximity(){}

    public boolean getShooterProximity(){
        return this.shooterProximity.get();
    }

    public boolean getCarriageProximity(){
        return this.carriageProximity.get();
    }
     
    public boolean getFeederProximity(){
        return this.feederProximity.get();
    }

    public void periodic(){
        // SmartDashboard.putBoolean("Shooter Proximity", getShooterProximity());
        // SmartDashboard.putBoolean("Carriage Proximity", getCarriageProximity());
        // SmartDashboard.putBoolean("Feeder Proximity", getFeederProximity());
    }
}