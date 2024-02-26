package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.Constants;

public class TOF {
    private final TimeOfFlight feederTOF = new TimeOfFlight(Constants.CANInfo.FEEDER_TOF_ID);
    private final TimeOfFlight carriageTOF = new TimeOfFlight(Constants.CANInfo.CARRIAGE_TOF_ID);
    private final TimeOfFlight intakeTOF = new TimeOfFlight(Constants.CANInfo.INTAKE_TOF_ID);

    public TOF(){
        this.feederTOF.setRangingMode(RangingMode.Short, 0);
        this.feederTOF.setRangeOfInterest(12, 16, 16, 0);
        this.carriageTOF.setRangingMode(RangingMode.Short, 25);
        this.intakeTOF.setRangingMode(RangingMode.Medium, 25);
    }

    public double getFeederDistMillimeters(){
        return this.feederTOF.getRange();
    }

    public double getCarriageDistMillimeters(){
        return this.carriageTOF.getRange();
    }

    public double getIntakeDistMillimeters(){
        return this.intakeTOF.getRange();
    }
}
