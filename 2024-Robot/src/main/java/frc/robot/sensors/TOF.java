package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class TOF {
    private final TimeOfFlight feederTOF = new TimeOfFlight(0);
    private final TimeOfFlight intakeTOF = new TimeOfFlight(1);

    public TOF(){
        this.feederTOF.setRangingMode(RangingMode.Short, 50);
        this.intakeTOF.setRangingMode(RangingMode.Medium, 50);
    }

    public double getFeederDistMillimeters(){
        return this.feederTOF.getRange();
    }

    public double getIntakeDistMillimeters(){
        return this.intakeTOF.getRange();
    }
}
