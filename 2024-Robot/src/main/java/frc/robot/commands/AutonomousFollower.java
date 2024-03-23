package frc.robot.commands;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

import org.json.JSONArray;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.tools.math.Vector;
import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;

public class AutonomousFollower extends Command {
  private Drive drive;
  private Lights lights;
  private Peripherals peripherals;
  private Proximity proximity;

  private JSONArray path;

  private double initTime;
  private double currentTime;
  private double previousTime;

  private double odometryFusedX = 0;
  private double odometryFusedY = 0;
  private double odometryFusedTheta = 0;

  private double[] desiredVelocityArray = new double[3];
  private double desiredThetaChange = 0;

  private boolean record;

  private ArrayList<double[]> recordedOdometry = new ArrayList<double[]>();
  private double pathStartTime;
  private double pathEndTime;

  private boolean pickupNote;
  private double noteTrackingEndTime;

  public AutonomousFollower(Drive drive, Lights lights, Peripherals peripherals, JSONArray pathPoints, double pathStartTime, double pathEndTime, boolean record, boolean pickupNote, double noteTrackingEndTime, Proximity proximity) {
    this.drive = drive;
    this.lights = lights;
    this.path = pathPoints;
    this.record = record;
    this.pathStartTime = pathStartTime;
    this.pathEndTime = pathEndTime;
    this.pickupNote = pickupNote;
    this.noteTrackingEndTime = noteTrackingEndTime;
    this.peripherals = peripherals;
    this.proximity = proximity;
    addRequirements(drive);
  }

  public AutonomousFollower(Drive drive, Lights lights, Peripherals peripherals, JSONArray pathPoints, double pathStartTime, boolean record, boolean pickupNote, double noteTrackingEndTime, Proximity proximity) {
    this.drive = drive;
    this.path = pathPoints;
    this.record = record;
    this.pathStartTime = pathStartTime;
    this.pathEndTime = path.getJSONArray(path.length() - 1).getDouble(0);
    this.pickupNote = pickupNote;
    this.noteTrackingEndTime = noteTrackingEndTime;
    this.lights = lights;
    this.peripherals = peripherals;
    this.proximity = proximity;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
    if(pickupNote) {
      lights.clearAnimations();
      lights.setCommandRunning(true);
      lights.setStrobePurple();
    }
  }

  @Override
  public void execute() {
    // System.out.println("Auto Follower");
    if(pickupNote && peripherals.getBackCamTrack()) {
      lights.setStrobeGreen();
    }
    // System.out.println("auto runs");
    drive.updateOdometryFusedArray();
    odometryFusedX = drive.getFusedOdometryX();
    odometryFusedY = drive.getFusedOdometryY();
    odometryFusedTheta = drive.getFusedOdometryTheta();
    // System.out.println("Follower field side: " + this.drive.getFieldSide());

    // System.out.println("Odom - X: " + odometryFusedX + " Y: " + odometryFusedY + " Theta: " + odometryFusedTheta);

    currentTime = Timer.getFPGATimestamp() - initTime + pathStartTime;
    
    if (proximity.getFeederProximity()){
      pickupNote = false;
      // System.out.println("sensor timeout");
    }
    // call PIDController function
    desiredVelocityArray = drive.pidController(odometryFusedX, odometryFusedY, odometryFusedTheta, currentTime, path, pickupNote, noteTrackingEndTime);
    
    // create velocity vector and set desired theta change
    Vector velocityVector = new Vector();
    velocityVector.setI(desiredVelocityArray[0]);
    velocityVector.setJ(desiredVelocityArray[1]);
    desiredThetaChange = desiredVelocityArray[2];
    // velocityVector.setI(0);
    // velocityVector.setJ(0);
    // desiredThetaChange = 0;

    drive.autoDrive(velocityVector, desiredThetaChange);

    this.previousTime = currentTime;

    if (this.record){
      recordedOdometry.add(new double[] {currentTime, odometryFusedX, odometryFusedY, odometryFusedTheta});
    }
  }

  @Override
  public void end(boolean interrupted) {
    if(pickupNote) {
      lights.clearAnimations();
      lights.setCommandRunning(false);
    }
    Vector velocityVector = new Vector();
    velocityVector.setI(0);
    velocityVector.setJ(0);
    double desiredThetaChange = 0.0;
    drive.autoDrive(velocityVector, desiredThetaChange);

    odometryFusedX = drive.getFusedOdometryX();
    odometryFusedY = drive.getFusedOdometryY();
    odometryFusedTheta = drive.getFusedOdometryTheta();
    currentTime = Timer.getFPGATimestamp() - initTime;

    if (this.record){
      recordedOdometry.add(new double[] {currentTime, odometryFusedX, odometryFusedY, odometryFusedTheta});

      try {
        DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy-MM-dd-hh-mm-ss");
        LocalDateTime now = LocalDateTime.now();
        String filename = "/home/lvuser/deploy/recordings/" + dtf.format(now) + ".csv";
        File file = new File(filename);
        if (!file.exists()){
          file.createNewFile();
        }
        FileWriter fw = new FileWriter(file);
        BufferedWriter bw = new BufferedWriter(fw);
        for (int i = 0; i <recordedOdometry.size(); i ++){
          String line = "";
          for (double val : recordedOdometry.get(i)){
            line += val + ",";
          }
          line = line.substring(0, line.length() - 1);
          line += "\n";
          bw.write(line);
        }
        
        bw.close();
      } catch (Exception e) {
        System.out.println(e);
        System.out.println("CSV file error");
      }
    }
  }

  @Override
  public boolean isFinished() {
    if(currentTime > this.pathEndTime) {
      return true;
    }
    else {
      return false;
    }
  }
}
