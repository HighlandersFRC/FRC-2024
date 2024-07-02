package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class AutoShoot extends Command {
  public static boolean canSeeTag;
  private Drive drive;
  private Shooter shooter;
  private Feeder feeder;
  private Peripherals peripherals;
  private Lights lights;
  private Proximity proximity;

  private double[] shooterValues;
  private double shooterDegrees;
  private double shooterRPM;
  private double feederRPM;

  private double startTime;
  private double timeout = 20;

  private double shotTime = 0;
  private double reachedSetPointTime = 0;
  private boolean hasShot;
  private double shotPauseTime = 0.0;

  private double shooterDegreesAllowedError = 0.75;
  private double shooterRPMAllowedError = 100;
  private double driveAngleAllowedError = 2;

  private double lookAheadTime = 0.0;

  private boolean hasReachedSetPoint;
  private int numTimesHitSetPoint;

  private PID pid;

  private double kP = 0.04;
  private double kI = 0;
  private double kD = 0.06;

  private double speakerElevationDegrees;
  private double speakerAngleDegrees;
  private double targetPigeonAngleDegrees;

  private double defaultShooterAngle = 0;
  private double defaultFlywheelRPM = 0;

  private ArrayList<double[]> tagReadings = new ArrayList<double[]>();
  private ArrayList<Double> pigeonAngles = new ArrayList<Double>();

  private boolean haveGoodSetpoint = false;
  private boolean auto = false;
  private int numTimesNoTrack = 0;

  public AutoShoot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, Proximity proximity, double feederRPM, double defaultShooterAngle, double defaultFlywheelRPM, boolean auto) {
    this.drive = drive;
    this.shooter = shooter;
    this.feeder = feeder;
    this.peripherals = peripherals;
    this.lights = lights;
    this.proximity = proximity;
    this.feederRPM = feederRPM;
    this.defaultShooterAngle = defaultShooterAngle;
    this.defaultFlywheelRPM = defaultFlywheelRPM;
    this.auto = auto;
    addRequirements(this.drive, this.shooter, this.feeder, this.lights);
  }

  public AutoShoot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, Proximity proximity, double feederRPM, double defaultShooterAngle, double defaultFlywheelRPM, double timeout, boolean auto) {
    this.drive = drive;
    this.shooter = shooter;
    this.feeder = feeder;
    this.peripherals = peripherals;
    this.lights = lights;
    this.proximity = proximity;
    this.feederRPM = feederRPM;
    this.defaultShooterAngle = defaultShooterAngle;
    this.defaultFlywheelRPM = defaultFlywheelRPM;
    this.timeout = timeout;
    this.auto = auto;
    this.numTimesNoTrack = 0;
    addRequirements(this.drive, this.shooter, this.feeder, this.lights);
  }

  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
    this.pigeonAngles = new ArrayList<Double>();
    this.hasShot = false;
    this.hasReachedSetPoint = false;
    this.haveGoodSetpoint = false;
    pid = new PID(kP, kI, kD);
    pid.setMinOutput(-3);
    pid.setMaxOutput(3);
    this.reachedSetPointTime = 0;
    this.numTimesHitSetPoint = 0;
    this.speakerElevationDegrees = 0;
    this.speakerAngleDegrees = 0;
    this.shooterValues = Constants.SetPoints.getShooterValuesFromAngle(this.speakerElevationDegrees);
    this.shooterDegrees = this.shooterValues[0];
    this.shooterRPM = this.shooterValues[1];
    this.tagReadings = new ArrayList<double[]>();
    lights.setCommandRunning(true);
    lights.setStrobePurple();

    this.targetPigeonAngleDegrees = this.peripherals.getPigeonAngle() - this.peripherals.getFrontCamTargetTx();
  }

  @Override
  public void execute() {
    // System.out.println("Auto Shoot");

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
      this.speakerElevationDegrees = this.peripherals.getFrontCamTargetTy();
      this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
      if (this.speakerElevationDegrees != 0.0 && this.speakerAngleDegrees != 0.0){
        this.shooterValues = Constants.SetPoints.getShooterValuesFromAngle(this.speakerElevationDegrees);
        this.shooterDegrees = this.shooterValues[0];
        this.shooterRPM = this.shooterValues[1];
        // System.out.println("New RPM: " + this.shooterRPM);
        // System.out.println("New Deg.: " + this.shooterDegrees);
        this.shooterDegreesAllowedError = Constants.SetPoints.getAllowedAngleErrFromAngle(this.speakerElevationDegrees);
        this.driveAngleAllowedError = Constants.SetPoints.getAllowedDriveAngleErrFromAngle(this.speakerElevationDegrees);
      }
      // double initialTargetPigeonAngleDegrees = pigeonAngleDegrees - this.speakerAngleDegrees;
      // System.out.println("Initial Degrees: " + initialTargetPigeonAngleDegrees);
      // this.targetPigeonAngleDegrees = Constants.SetPoints.getAdjustedPigeonAngle(initialTargetPigeonAngleDegrees, Constants.SetPoints.getDistFromAngle(this.speakerElevationDegrees));
      // this.targetPigeonAngleDegrees = initialTargetPigeonAngleDegrees;
      // System.out.println("Adjusted Degrees: " + this.targetPigeonAngleDegrees);
    }

    //get current apriltag track
    double tagElevationDegrees = this.peripherals.getFrontCamTargetTy();
    double tagAngleDegrees = this.peripherals.getFrontCamTargetTx();
    double tagReadingHB = this.peripherals.getFrontCamHB();
    double tagLatency = this.peripherals.getFrontCameraLatency();

    this.pigeonAngles.add(pigeonAngleDegrees);
    if (this.pigeonAngles.size() > 10){
      this.pigeonAngles.remove(0);
    }

    int lookback = (int) ((tagLatency / 20.0));
    if (lookback >= this.pigeonAngles.size()){
      // System.out.println("over: " + lookback);
      lookback = this.pigeonAngles.size() - 1;
    }
    // System.out.println("size: " + this.pigeonAngles.size());
    // System.out.println("lookback: " + (this.pigeonAngles.size() - lookback - 1));
    double pigeonAngleOffset = this.pigeonAngles.get(this.pigeonAngles.size() - lookback - 1) - pigeonAngleDegrees;
    // System.out.println("Offset: " + pigeonAngleOffset);

    if (canSeeTag){
      lights.setStrobeGreen();
      double initialTargetPigeonAngleDegrees = pigeonAngleDegrees - this.speakerAngleDegrees + pigeonAngleOffset;
      this.targetPigeonAngleDegrees = initialTargetPigeonAngleDegrees;
    } else {
      lights.setStrobePurple();
    }

    // System.out.println("Targ 1: " + (pigeonAngleDegrees - this.speakerAngleDegrees));
    // System.out.println("Targ 2: " + (pigeonAngleDegrees - this.speakerAngleDegrees + pigeonAngleOffset));

    // if (tagAngleDegrees < 90){
    //   this.speakerAngleDegrees = tagAngleDegrees;
    // }

    // //add track to list and trim size if over 10
    if (this.tagReadings.size() == 0){
      // System.out.println("1");
      this.tagReadings.add(new double[] {tagElevationDegrees, tagAngleDegrees, tagReadingHB});
    } else if (this.tagReadings.get(this.tagReadings.size() - 1)[2] != tagReadingHB && tagReadingHB != -1){
      // System.out.println("2");
      this.tagReadings.add(new double[] {tagElevationDegrees, tagAngleDegrees, tagReadingHB});
    } else {
      // System.out.println("3");
    }
    // System.out.println("HB: " + tagReadingHB);
    // System.out.println("-----------");

    if (this.tagReadings.size() > 10){
      this.tagReadings.remove(0);
    }

    // //add up valid tracks
    int numValidTags = 0;
    double avgElev = 0;
    double avgAng = 0;
    for (int i = 0; i < this.tagReadings.size(); i ++){
      double elev = this.tagReadings.get(i)[0];
      double ang = this.tagReadings.get(i)[1];
      // System.out.println("A: " + ang);
      // System.out.println("E: " + elev);
      if (elev != 0.0 && ang != 0.0){
        numValidTags ++;
        avgElev += elev;
        avgAng += ang;
      }
    }

    // System.out.println("Num Valid: " + numValidTags);

    // //average tracks, discard if not enough tracks
    // if (numValidTags > 0){
    //   this.shooterValues = Constants.SetPoints.getShooterValuesFromAngle(avgElev / numValidTags);
    //   this.shooterDegrees = this.shooterValues[0];
    //   this.shooterRPM = this.shooterValues[1];
    //   // System.out.println("Avg Elev: " + (avgElev / numValidTags));
    // }

    if (numValidTags < 5){
      this.haveGoodSetpoint = false;
    } else {
      this.haveGoodSetpoint = true;
    }
    // this.haveGoodSetpoint = true;

    // System.out.println("Good Setpoint: " + this.haveGoodSetpoint);

    if (canSeeTag){
      this.pid.setSetPoint(this.targetPigeonAngleDegrees);
      this.pid.updatePID(pigeonAngleDegrees);
    } else {
      numTimesNoTrack++;
      // System.out.println("running");
      if (!this.auto){
        this.pid.setSetPoint(180);
        this.pid.updatePID(Constants.SetPoints.standardizeAngleDegrees(pigeonAngleDegrees));
      }
    }
    double turnResult = -pid.getResult();

    if (Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(pigeonAngleDegrees - this.targetPigeonAngleDegrees) <= this.driveAngleAllowedError){
      this.numTimesHitSetPoint ++;
    } else {
      this.numTimesHitSetPoint = 0;
    }

    if (this.numTimesHitSetPoint >= 2 && Math.abs(this.shooter.getFlywheelMasterRPM() - this.shooterRPM) <= this.shooterRPMAllowedError && Math.abs(this.shooter.getFlywheelFollowerRPM() - this.shooterRPM) <= this.shooterRPMAllowedError){
      this.hasReachedSetPoint = true;
      this.reachedSetPointTime = Timer.getFPGATimestamp();
      turnResult = 0;
    }

    // System.out.println("Turn result: " + turnResult);
    if (this.speakerAngleDegrees < 90){
      // System.out.println("1");
      this.drive.driveAutoAligned(turnResult);
    } else {
      // System.out.println("2");
      this.drive.driveAutoAligned(0);
    }

    if (this.proximity.getFeederProximity()){
      this.shooter.set(this.shooterDegrees, this.shooterRPM);
    }    
    
    if (this.hasReachedSetPoint == true && this.haveGoodSetpoint){
      lights.clearAnimations();
      lights.setCandleRGB(0, 255, 0);
      // System.out.println("Shooting");
      this.feeder.set(this.feederRPM);
    } else {
      this.feeder.set(0.0);
    }

    if (!this.proximity.getFeederProximity() && !this.hasShot){
      this.hasShot = true;
      this.shotTime = Timer.getFPGATimestamp();
      // System.out.println("Has Shot");
    }

    if (Timer.getFPGATimestamp() - this.startTime >= this.timeout){
      this.hasReachedSetPoint = true;
    }

    // System.out.println("Num Times Hit Setpoint: " + this.numTimesHitSetPoint);
    // System.out.println("Angle: " + Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees));
    // System.out.println("RPM: " + Math.abs(this.shooter.getFlywheelMasterRPM() - this.shooterRPM));
    // System.out.println("Elev: " + Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees));
    // System.out.println("Master RPM: " + this.shooter.getFlywheelMasterRPM());
    // System.out.println("Follower RPM: " + this.shooter.getFlywheelFollowerRPM());
    // System.out.println("Targ. RPM: " + this.shooterRPM);
    // System.out.println("RPM Err: " + Math.abs(this.shooter.getFlywheelRPM() - this.shooterRPM));
    // System.out.println("Elev: " + this.shooter.getAngleDegrees());
    // System.out.println("Targ. Elev: " + this.shooterDegrees);
    // System.out.println("Elev Err: " + Math.abs(this.shooter.getAngleDegrees() - shooterDegrees));
    // System.out.println("Pigeon Angle: " + pigeonAngleDegrees);
    // System.out.println("Targ. Pigeon Angle: " + this.targetPigeonAngleDegrees);
    // System.out.println("Pigeon Angle Err: " + Math.abs(pigeonAngleDegrees - this.targetPigeonAngleDegrees));
    // System.out.println("Speaker Ang Deg: " + this.speakerAngleDegrees);
    // System.out.println("Speaker Elev Deg: " + this.speakerElevationDegrees);
    // System.out.println("<================>");
    // System.out.println(pigeonAngleDegrees + "," + this.targetPigeonAngleDegrees + "," + this.speakerAngleDegrees);
    // System.out.println("latency: " + this.peripherals.getFrontCameraLatency());
  }

  @Override
  public void end(boolean interrupted) {
    this.feeder.set(0);
    this.shooter.setAngleTorque(-10, 0.3);
    lights.clearAnimations();
    lights.setCommandRunning(false);
  }

  @Override
  public boolean isFinished() {
    // System.out.println("Duration " + (Timer.getFPGATimestamp() - this.startTime));
    // System.out.println("Has Shot " + this.hasShot);
    // System.out.println("Time Since Shot " + (Timer.getFPGATimestamp() - this.shotTime));

    if (this.shooterDegrees > 90){
      return true;
    } else if (this.hasShot && Timer.getFPGATimestamp() - this.shotTime >= this.shotPauseTime){
      return true;
    } else if (Timer.getFPGATimestamp() - this.startTime > this.timeout + 1.0){
      return true;
    } else {
      return false;
    }
  }
}