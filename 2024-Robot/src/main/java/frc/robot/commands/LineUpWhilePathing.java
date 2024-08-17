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
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;
import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;

public class LineUpWhilePathing extends Command {
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

  private PID pid;
  private double kP = 0.04;
  private double kI = 0;
  private double kD = 0.06;
  public static boolean canSeeTag;
  private double speakerElevationDegrees;
  private double speakerAngleDegrees;
  private double targetPigeonAngleDegrees;

  public LineUpWhilePathing(Drive drive, Lights lights, Peripherals peripherals, JSONArray pathPoints, double pathStartTime, double pathEndTime, boolean record, boolean pickupNote, double noteTrackingEndTime, Proximity proximity) {
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

  public LineUpWhilePathing(Drive drive, Lights lights, Peripherals peripherals, JSONArray pathPoints, double pathStartTime, boolean record, boolean pickupNote, double noteTrackingEndTime, Proximity proximity) {
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
    pid = new PID(kP, kI, kD);
    pid.setMinOutput(-3);
    pid.setMaxOutput(3);
  }

  @Override
  public void execute() {
    double pigeonAngleDegrees = this.peripherals.getPigeonAngle();

    double id = this.peripherals.getFrontCamID();

    canSeeTag = false;
    // for (double id : ids){
      if (id == 7 || id == 4){
        canSeeTag = true;
      }
    // }

    // System.out.println("Can See Tag: " + canSeeTag);

    if (canSeeTag){
      lights.setStrobeGreen();
      this.speakerElevationDegrees = this.peripherals.getFrontCamTargetTy();
      this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
    }

    if (canSeeTag){
      double initialTargetPigeonAngleDegrees = pigeonAngleDegrees - this.speakerAngleDegrees;
      this.targetPigeonAngleDegrees = initialTargetPigeonAngleDegrees;
    }

    this.pid.setSetPoint(this.targetPigeonAngleDegrees);
    this.pid.updatePID(pigeonAngleDegrees);
    double turnResult = -pid.getResult();
    if(pickupNote && peripherals.getBackCamTrack()) {
      lights.setStrobeGreen();
    }
    drive.updateOdometryFusedArray();
    odometryFusedX = drive.getFusedOdometryX();
    odometryFusedY = drive.getFusedOdometryY();
    odometryFusedTheta = drive.getFusedOdometryTheta();
    // odometryFusedX = drive.getMT2OdometryX();
    // odometryFusedY = drive.getMT2OdometryY();
    // odometryFusedTheta = drive.getMT2OdometryAngle();
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

    if (canSeeTag && this.proximity.getFeederProximity() && this.speakerAngleDegrees < 90){
      // System.out.println("1");
      drive.autoDrive(velocityVector, turnResult);
    } else {
      // System.out.println("2");
      drive.autoDrive(velocityVector, desiredThetaChange);
    }

    this.previousTime = currentTime;

    if (this.record){
      recordedOdometry.add(new double[] {currentTime, odometryFusedX, odometryFusedY, odometryFusedTheta});
    }
    // System.out.println("Pigeon Angle: " + pigeonAngleDegrees);
    // System.out.println("Targ. Pigeon Angle: " + this.targetPigeonAngleDegrees);
    // System.out.println("Pigeon Angle Err: " + Math.abs(pigeonAngleDegrees - this.targetPigeonAngleDegrees));
    // System.out.println("Speaker Ang Deg: " + this.speakerAngleDegrees);
    // System.out.println("Speaker Elev Deg: " + this.speakerElevationDegrees);
    // System.out.println("<================>");
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
