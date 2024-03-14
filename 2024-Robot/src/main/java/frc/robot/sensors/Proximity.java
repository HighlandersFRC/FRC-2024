package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Proximity {
    public final DigitalInput shooterProximity = new DigitalInput(Constants.CANInfo.SHOOTER_PROXIMITY_PORT);

    public Proximity(){}

    public boolean getShooterProximity(){
        return this.shooterProximity.get();
    }

    public void periodic(){
        SmartDashboard.putBoolean("Shooter Proximity", getShooterProximity());
    }
}