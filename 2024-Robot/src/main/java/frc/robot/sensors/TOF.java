package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.Constants;

public class TOF {
    public static final TimeOfFlight feederTOF = new TimeOfFlight(Constants.CANInfo.FEEDER_TOF_ID);
    public static final TimeOfFlight intakeTOF = new TimeOfFlight(Constants.CANInfo.INTAKE_TOF_ID);
    public static final TimeOfFlight climberTOF = new TimeOfFlight(Constants.CANInfo.CLIMBER_TOF_ID);

    public TOF(){
        TOF.feederTOF.setRangingMode(RangingMode.Short, 50);
        TOF.intakeTOF.setRangingMode(RangingMode.Short, 50);
        TOF.climberTOF.setRangingMode(RangingMode.Short, 50);

    }

    public double getFeederDistMillimeters(){
        return TOF.feederTOF.getRange();
    }

    public double getIntakeDistMillimeters(){
        return TOF.intakeTOF.getRange();
    }

    public double getClimberDistMillimeters(){
        return TOF.climberTOF.getRange();
    }
}
