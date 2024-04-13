package frc.robot.sensors;

import java.util.ArrayList;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TOF {
    // public final TimeOfFlight feederTOF = new TimeOfFlight(Constants.CANInfo.FEEDER_TOF_ID);
    public final TimeOfFlight intakeTOF = new TimeOfFlight(Constants.CANInfo.INTAKE_TOF_ID);
    // public final TimeOfFlight carriageTOF = new TimeOfFlight(Constants.CANInfo.CARRIAGE_TOF_ID);

    private ArrayList<Double> prevIntakeDists = new ArrayList<Double>();

    public TOF(){
        // this.feederTOF.setRangingMode(RangingMode.Short, 0);
        // this.feederTOF.setRangeOfInterest(14, 16, 16, 0);

        //Probably positioned towards the inside of the intake, centered between top and bottom
        this.intakeTOF.setRangingMode(RangingMode.Short, 0);
        this.intakeTOF.setRangeOfInterest(6, 1, 10, 5);
        this.prevIntakeDists.add(0.0);
        this.prevIntakeDists.add(1.0);
        // this.carriageTOF.setRangingMode(RangingMode.Short, 0);
        // this.carriageTOF.setRangeOfInterest(6, 8, 8, 6);
    }

    public boolean isIntakeTOFConnected(){
        this.updateIntakeDist();
        for (int i = 1; i < this.prevIntakeDists.size(); i ++){
            if (this.prevIntakeDists.get(i).doubleValue() != this.prevIntakeDists.get(i - 1).doubleValue()){
                return true;
            }
        }
        return false;
    }

    public double getFeederDistMillimeters(){
        // return this.feederTOF.getRange();
        return 0;
    }

    public double getCarriageDistMillimeters(){
        // return this.carriageTOF.getRange();
        return 0;
    }

    public double getIntakeDistMillimeters(){
        this.updateIntakeDist();
        return this.intakeTOF.getRange();
    }

    public void updateIntakeDist(){
        double dist = this.intakeTOF.getRange();
        this.prevIntakeDists.add(dist);
        if (this.prevIntakeDists.size() > 10){
            this.prevIntakeDists.remove(0);
        }
    }

    public void periodic(){
        // boolean climbTOF = false;
        // if (this.carriageTOF.getRange() > 0 && this.carriageTOF.getRange() < 1000.0){
        //     climbTOF = true;
        // }
        // boolean feederTOF = false;
        // if (this.feederTOF.getRange() > 0 && this.feederTOF.getRange() < 1000.0){
        //     feederTOF = true;
        // }
        // boolean intakeTOF = false;
        // if  (this.intakeTOF.getRange() > 0 && this.intakeTOF.getRange() < 1000.0){
        //     intakeTOF = true;
        // }

        // SmartDashboard.putBoolean(" Climber TOF", climbTOF);
        // SmartDashboard.putBoolean(" Feeder TOF", feederTOF);
        SmartDashboard.putNumber(" Intake TOF", this.intakeTOF.getRange());
        SmartDashboard.putBoolean("Intake Bool", this.intakeTOF.getRange() < Constants.SetPoints.INTAKE_TOF_THRESHOLD_MM);
        // System.out.println("dist: " + this.intakeTOF.getRange());
        // System.out.println("bool: " + (this.intakeTOF.getRange() < Constants.SetPoints.INTAKE_TOF_THRESHOLD_MM));
        // System.out.println(this.isIntakeTOFConnected());
        // SmartDashboard.putNumber("Feeder TOF Dist", getFeederDistMillimeters());
        // SmartDashboard.putNumber("Carriage TOF Dist", getCarriageDistMillimeters());
    }
}