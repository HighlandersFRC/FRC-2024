package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.Constants;

public class TOF {
    private final TimeOfFlight feederTOF = new TimeOfFlight(0);

    public TOF(){
        this.feederTOF.setRangingMode(RangingMode.Short, 50);
    }

    public double getFeederDistMillimeters(){
        return TOF.feederTOF.getRange();
    }

    public double getIntakeDistMillimeters(){
        return this.intakeTOF.getRange();
    }
}
