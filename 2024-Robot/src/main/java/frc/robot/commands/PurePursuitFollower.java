package frc.robot.commands;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.tools.math.Vector;
import frc.robot.tools.wrappers.PolarTakeDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;

public class PurePursuitFollower extends PolarTakeDrive{
  private Drive drive;
  private Lights lights;
  private Peripherals peripherals;

  private JSONArray path;

  private double initTime;
  private double currentTime;

  private double odometryFusedX = 0;
  private double odometryFusedY = 0;
  private double odometryFusedTheta = 0;

  private Number[] desiredVelocityArray = new Number[4];
  private double desiredThetaChange = 0;

  private boolean record;

  private ArrayList<double[]> recordedOdometry = new ArrayList<double[]>();
  public double pathStartTime;

  private boolean pickupNote;

  private int currentPathPointIndex = 0;
  private int returnPathPointIndex = 0;
  private int timesStagnated = 0;
  private final int STAGNATE_THRESHOLD = 3;
  private boolean reset = true;

  public int getPathPointIndex() {
    return currentPathPointIndex;
  }

  public PurePursuitFollower(Drive drive, Lights lights, Peripherals peripherals, JSONArray pathPoints,
      boolean record) {
    this.drive = drive;
    this.lights = lights;
    this.path = pathPoints;
    this.record = record;
    pathStartTime = pathPoints.getJSONObject(0).getDouble("time");
    this.peripherals = peripherals;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
    if (reset) {
      currentPathPointIndex = 0;
    }else {
      reset = true;
    }
    returnPathPointIndex = currentPathPointIndex;
    timesStagnated = 0;
    if (pickupNote) {
      lights.clearAnimations();
      lights.setCommandRunning(true);
      lights.setStrobePurple();
    }
  }

  @Override
  public void execute() {
    if (pickupNote && peripherals.getBackCamTrack()) {
      lights.setStrobeGreen();
    }
    drive.updateOdometryFusedArray();
    odometryFusedX = drive.getMT2OdometryX();
    odometryFusedY = drive.getMT2OdometryY();
    odometryFusedTheta = drive.getMT2OdometryAngle();
    currentTime = Timer.getFPGATimestamp() - initTime + pathStartTime;
    // call PIDController function
    currentPathPointIndex = returnPathPointIndex;
    desiredVelocityArray = drive.purePursuitController(odometryFusedX, odometryFusedY, odometryFusedTheta,
        currentPathPointIndex, path);

    returnPathPointIndex = desiredVelocityArray[3].intValue();
    if (returnPathPointIndex == currentPathPointIndex) {
      timesStagnated++;
      if (timesStagnated > STAGNATE_THRESHOLD) {
        returnPathPointIndex++;
        timesStagnated = 0;
      }
    } else {
      timesStagnated = 0;
    }

    // create velocity vector and set desired theta change
    Vector velocityVector = new Vector();
    velocityVector.setI(desiredVelocityArray[0].doubleValue());
    velocityVector.setJ(desiredVelocityArray[1].doubleValue());
    desiredThetaChange = desiredVelocityArray[2].doubleValue();

    drive.autoDrive(velocityVector, desiredThetaChange);
    Logger.recordOutput("pursuing?", true);
    Logger.recordOutput("Path Time", path
        .getJSONObject(getPathPointIndex()).getDouble("time"));
  }

  @Override
  public void end(boolean interrupted) {
    if (pickupNote) {
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
    Logger.recordOutput("pursuing?", false);
    if (this.record) {
      recordedOdometry.add(new double[] { currentTime, odometryFusedX, odometryFusedY, odometryFusedTheta });
      try {
        DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy-MM-dd-hh-mm-ss");
        LocalDateTime now = LocalDateTime.now();
        String filename = "/home/lvuser/deploy/recordings/" + dtf.format(now) + ".csv";
        File file = new File(filename);
        if (!file.exists()) {
          file.createNewFile();
        }
        FileWriter fw = new FileWriter(file);
        BufferedWriter bw = new BufferedWriter(fw);
        for (int i = 0; i < recordedOdometry.size(); i++) {
          String line = "";
          for (double val : recordedOdometry.get(i)) {
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

  public void from(int pointIndex, JSONObject pathJSON, int to) {
    this.currentPathPointIndex = pointIndex;
    path = pathJSON.getJSONArray("sampled_points");
    reset = false;
    this.schedule();
  }

  @Override
  public boolean isFinished() {
    if (returnPathPointIndex >= path.length() - 1) {
      return true;
    } else {
      return false;
    }
  }
}
