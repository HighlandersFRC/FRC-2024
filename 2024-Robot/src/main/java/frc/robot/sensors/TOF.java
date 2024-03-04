package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.Constants;

public class TOF {
    public static final TimeOfFlight feederTOF = new TimeOfFlight(Constants.CANInfo.FEEDER_TOF_ID);
    public static final TimeOfFlight intakeTOF = new TimeOfFlight(Constants.CANInfo.INTAKE_TOF_ID);
    public static final TimeOfFlight carriageTOF = new TimeOfFlight(Constants.CANInfo.CARRIAGE_TOF_ID);

    public TOF(){
        TOF.feederTOF.setRangingMode(RangingMode.Short, 50);
        TOF.intakeTOF.setRangingMode(RangingMode.Short, 50);
        TOF.carriageTOF.setRangingMode(RangingMode.Short, 50);

    }

    public double getFeederDistMillimeters(){
        return TOF.feederTOF.getRange();
    }

    public double getCarriageDistMillimeters(){
        return TOF.carriageTOF.getRange();
    }

    public double getIntakeDistMillimeters(){
        return TOF.intakeTOF.getRange();
    }

}
