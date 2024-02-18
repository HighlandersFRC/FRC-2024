package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TOF {
    private final TimeOfFlight feederTOF = new TimeOfFlight(0);
    private final TimeOfFlight intakeTOF = new TimeOfFlight(1);
    private boolean feederBool = false;
    private boolean intakeBool = false;

    public TOF(){
        this.feederTOF.setRangingMode(RangingMode.Short, 50);
       if(feederTOF.getRange() != 0){
        feederBool = true;
       } 
       this.intakeTOF.setRangingMode(RangingMode.Short, 50);
       if(intakeTOF.getRange() != 0){
        intakeBool = true;
        }

       SmartDashboard.putBoolean(" intake tof", intakeBool);
       SmartDashboard.putBoolean(" feeder tof", feederBool);
    }

    public double getFeederDistMillimeters(){
        return this.feederTOF.getRange();
    }
}
