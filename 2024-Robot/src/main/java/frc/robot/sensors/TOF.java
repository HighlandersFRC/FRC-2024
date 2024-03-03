package frc.robot.sensors;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TOF {
    public final TimeOfFlight feederTOF = new TimeOfFlight(Constants.CANInfo.FEEDER_TOF_ID);
    public final TimeOfFlight intakeTOF = new TimeOfFlight(Constants.CANInfo.INTAKE_TOF_ID);
    public final TimeOfFlight carriageTOF = new TimeOfFlight(Constants.CANInfo.CARRIAGE_TOF_ID);

    public TOF(){
        this.feederTOF.setRangingMode(RangingMode.Short, 0);
        this.feederTOF.setRangeOfInterest(12, 16, 16, 0);
        this.intakeTOF.setRangingMode(RangingMode.Short, 50);
        this.carriageTOF.setRangingMode(RangingMode.Short, 50);
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

    public void periodic(){
        boolean climbTOF = false;
        if (this.carriageTOF.getRange() > 0 && this.carriageTOF.getRange() < 1000.0){
            climbTOF = true;
        }
        boolean feederTOF = false;
        if (this.feederTOF.getRange() > 0 && this.feederTOF.getRange() < 1000.0){
            feederTOF = true;
        }
        boolean intakeTOF = false;
        if  (this.intakeTOF.getRange() > 0 && this.intakeTOF.getRange() < 1000.0){
            intakeTOF = true;
        }

        SmartDashboard.putBoolean(" Climber TOF", climbTOF);
        SmartDashboard.putBoolean(" Feeder TOF", feederTOF);
        SmartDashboard.putBoolean(" Intake TOF", intakeTOF);
        SmartDashboard.putNumber("Feeder TOF Dist", getFeederDistMillimeters());
    }
}