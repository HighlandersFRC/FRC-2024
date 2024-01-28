package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class TOF {
    private final TimeOfFlight feederTOF = new TimeOfFlight(0);

    public TOF(){
        this.feederTOF.setRangingMode(RangingMode.Short, 50);
    }

    public double getFeederDistMillimeters(){
        return this.feederTOF.getRange();
    }
}
