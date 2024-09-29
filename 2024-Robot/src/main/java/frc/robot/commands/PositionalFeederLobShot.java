package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class PositionalFeederLobShot extends Command {
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
  private double distToSpeakerMeters;
  private double angleToSpeakerDegrees;
  private double angleOffset;

  private double defaultShooterAngle = 0;
  private double defaultFlywheelRPM = 0;

  private ArrayList<double[]> tagReadings = new ArrayList<double[]>();
  private ArrayList<Double> pigeonAngles = new ArrayList<Double>();

  private boolean haveGoodSetpoint = false;
  private boolean auto = false;
  private int numTimesNoTrack = 0;

  private double x;
  private double y;
  private double targetAngle;
  private double angleX;
  private double angleY;

  public PositionalFeederLobShot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, Proximity proximity, double feederRPM) {
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

  public PositionalFeederLobShot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, Proximity proximity, double feederRPM, double timeout) {
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
    this.shooterValues = Constants.SetPoints.getShooterValuesFromDistance(this.speakerElevationDegrees, true);
    this.shooterDegrees = this.shooterValues[0];
    this.shooterRPM = this.shooterValues[1];
    lights.setCommandRunning(true);
    lights.setStrobePurple();
    this.distToSpeakerMeters = 0;
    this.angleToSpeakerDegrees = 0;
    this.targetPigeonAngleDegrees = this.peripherals.getPigeonAngle() - this.peripherals.getFrontCamTargetTx();
    this.angleOffset = 0;

    if (drive.getFieldSide() == "red"){
      x = Constants.Physical.FIELD_LENGTH - Constants.Physical.FEEDER_LOB_SHOT_TARGET_X;
      y = Constants.Physical.FEEDER_LOB_SHOT_TARGET_Y;
    } else {
      x = Constants.Physical.FEEDER_LOB_SHOT_TARGET_X;
      y = Constants.Physical.FEEDER_LOB_SHOT_TARGET_Y;
    }
  }

  @Override
  public void execute() {
    // System.out.println("Auto Shoot");

    double pigeonAngleDegrees = this.peripherals.getPigeonAngle();

    //-0.5 to counter the note sliding after landing
    this.distToSpeakerMeters = Constants.getDistance(x, y, drive.getMT2OdometryX(), drive.getMT2OdometryY()) - 0.5;
    this.angleToSpeakerDegrees = Constants.getAngleToPoint(x, y, drive.getMT2OdometryX(), drive.getMT2OdometryY());
    this.shooterValues = Constants.SetPoints.getShooterValuesFromDistance(this.distToSpeakerMeters, true);
    this.shooterDegrees = this.shooterValues[0];
    this.shooterRPM = this.shooterValues[1];
    this.angleOffset = Constants.SetPoints.getRobotAngleOffset(shooterRPM);
    
    // System.out.println("dist: " + distToSpeakerMeters);
    // System.out.println("y: " + y);
    // System.out.println("angle: " + angleToSpeakerDegrees);
    // System.out.println("angle offset" + angleOffset);
    // System.out.println("deg" + shooterDegrees);
    // System.out.println("rpm" + shooterRPM);

    if (drive.getFieldSide() == "red"){
      targetAngle = angleToSpeakerDegrees + 180 - angleOffset;
    } else {
      targetAngle = angleToSpeakerDegrees - angleOffset;
    }

    // System.out.println("target angle" + targetAngle);

    if (DriverStation.isAutonomousEnabled() && drive.getFieldSide() == "red"){
      // System.out.println("autonoumous");
      pigeonAngleDegrees = 180 + pigeonAngleDegrees;
    }

    // System.out.println("angle: " + targetAngle);

    this.pid.setSetPoint(Constants.SetPoints.standardizeAngleDegrees(targetAngle));
    this.pid.updatePID(Constants.SetPoints.standardizeAngleDegrees(pigeonAngleDegrees));
    double turnResult = -pid.getResult();

    if (Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(Constants.SetPoints.standardizeAngleDegrees(pigeonAngleDegrees) - Constants.SetPoints.standardizeAngleDegrees(targetAngle)) <= this.driveAngleAllowedError){
      this.numTimesHitSetPoint ++;
    } else {
      this.numTimesHitSetPoint = 0;
    }

    if (this.numTimesHitSetPoint >= 2 && Math.abs(this.shooter.getFlywheelMasterRPM() - this.shooterRPM) <= this.shooterRPMAllowedError && Math.abs(this.shooter.getFlywheelFollowerRPM() - this.shooterRPM) <= this.shooterRPMAllowedError){
      this.hasReachedSetPoint = true;
      this.reachedSetPointTime = Timer.getFPGATimestamp();
      turnResult = 0;
    }

    this.drive.driveAutoAligned(turnResult);
    this.shooter.set(this.shooterDegrees, this.shooterRPM);
    
    if (this.hasReachedSetPoint == true){
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
    // System.out.println("Pigeon Angle Err: " + Math.abs(pigeonAngleDegrees - this.angleToSpeakerDegrees));
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
    } else if (OI.getPOV() == -1){
      return true;
    } else if (Timer.getFPGATimestamp() - this.startTime > this.timeout + 1.0){
      return true;
    } else {
      return false;
    }
  }
}