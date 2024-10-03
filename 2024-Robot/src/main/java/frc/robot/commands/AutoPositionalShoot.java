package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SetPoints.IntakePosition;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class AutoPositionalShoot extends Command {
  public static boolean canSeeTag;
  private Drive drive;
  private Shooter shooter;
  private Feeder feeder;
  private Peripherals peripherals;
  private Lights lights;

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

  public AutoPositionalShoot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, double feederRPM, double defaultShooterAngle, double defaultFlywheelRPM, boolean auto) {
    this.drive = drive;
    this.shooter = shooter;
    this.feeder = feeder;
    this.peripherals = peripherals;
    this.lights = lights;
    this.feederRPM = feederRPM;
    this.defaultShooterAngle = defaultShooterAngle;
    this.defaultFlywheelRPM = defaultFlywheelRPM;
    this.auto = auto;
    addRequirements(this.drive, this.shooter, this.feeder, this.lights);
  }

  public AutoPositionalShoot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, double feederRPM, double defaultShooterAngle, double defaultFlywheelRPM, double timeout, boolean auto) {
    this.drive = drive;
    this.shooter = shooter;
    this.feeder = feeder;
    this.peripherals = peripherals;
    this.lights = lights;
    this.feederRPM = feederRPM;
    this.defaultShooterAngle = defaultShooterAngle;
    this.defaultFlywheelRPM = defaultFlywheelRPM;
    this.timeout = timeout;
    this.auto = auto;
    this.numTimesNoTrack = 0;
    addRequirements(this.drive, this.shooter, this.feeder, this.lights);
  }

  public AutoPositionalShoot(Intake intake, Feeder feeder2, Lights lights2, IntakePosition kdown, int i, int j, boolean b, boolean c) {
    //TODO Auto-generated constructor stub
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
    this.distToSpeakerMeters = 0;
    this.angleToSpeakerDegrees = 0;
    this.targetPigeonAngleDegrees = this.peripherals.getPigeonAngle() - this.peripherals.getFrontCamTargetTx();

    if (drive.getFieldSide() == "red"){
      // System.out.println("-----------red------");
      x = Constants.Physical.FIELD_LENGTH;
      angleX = x - (Constants.Physical.SPEAKER_DEPTH / 2);
      angleY = Constants.Physical.SPEAKER_Y - 0.2;
    } else {
      // System.out.println("-----------blue------");
      x = Constants.Physical.SPEAKER_X;
      angleX = x + (Constants.Physical.SPEAKER_DEPTH / 2);
      angleY = Constants.Physical.SPEAKER_Y + 0.2;
    }

    if (drive.getMT2OdometryY() < 2.0){
      angleY -= 0.06;
    }
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
    // }]\[]
    

    this.distToSpeakerMeters = Constants.getDistance(x, Constants.Physical.SPEAKER_Y, drive.getMT2OdometryX(), drive.getMT2OdometryY());
    Logger.recordOutput("DistToSpeakerMeters", distToSpeakerMeters);
    this.angleToSpeakerDegrees = Constants.getAngleToPoint(angleX, angleY, drive.getMT2OdometryX(), drive.getMT2OdometryY());
    this.shooterValues = Constants.SetPoints.getShooterValuesFromDistance(this.distToSpeakerMeters);
    this.shooterDegrees = this.shooterValues[0];
    this.shooterRPM = this.shooterValues[1];
    this.shooterDegreesAllowedError = this.shooterValues[2];
    // System.out.println("dist: " + distToSpeakerMeters);
    // System.out.println("x: " + x);
    // System.out.println("angle: " + angleToSpeakerDegrees);
    // System.out.println("deg" + shooterDegrees);
    // System.out.println("rpm" + shooterRPM);

    if (drive.getFieldSide() == "red"){
      targetAngle = angleToSpeakerDegrees + 180;
    } else {
      targetAngle = angleToSpeakerDegrees;
    }

    if (DriverStation.isAutonomousEnabled() && drive.getFieldSide() == "red"){
      // System.out.println("autonoumous");
      pigeonAngleDegrees = 180 + pigeonAngleDegrees;
    }

    // System.out.println("angle: " + targetAngle);

    this.pid.setSetPoint(Constants.SetPoints.standardizeAngleDegrees(targetAngle));
    this.pid.updatePID(Constants.SetPoints.standardizeAngleDegrees(pigeonAngleDegrees));
    double turnResult = -pid.getResult();

    if (Math.abs(this.shooter.getShooterAngle() - this.shooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(Constants.SetPoints.standardizeAngleDegrees(pigeonAngleDegrees) - Constants.SetPoints.standardizeAngleDegrees(targetAngle)) <= this.driveAngleAllowedError){
      this.numTimesHitSetPoint ++;
    } else {
      this.numTimesHitSetPoint = 0;
    }

    if (this.numTimesHitSetPoint >= 2 && Math.abs(this.shooter.getLeftShooterRPM() - this.shooterRPM) <= this.shooterRPMAllowedError && Math.abs(this.shooter.getRightShooterRPM() - this.shooterRPM/2) <= this.shooterRPMAllowedError){
      this.hasReachedSetPoint = true;
      this.reachedSetPointTime = Timer.getFPGATimestamp();
      turnResult = 0;
    }

    this.drive.driveAutoAligned(turnResult);
    this.shooter.setShooterRPM(this.shooterRPM, this.shooterRPM/2);
    this.shooter.setShooterAngle(this.shooterDegrees);
    
    if (this.hasReachedSetPoint == true){
      lights.clearAnimations();
      lights.setCandleRGB(0, 255, 0);
      // System.out.println("Shooting");
      this.feeder.setRPM(this.feederRPM);
      this.hasShot = true;
      this.shotTime = Timer.getFPGATimestamp();
    } else {
      this.feeder.setRPM(0.0);
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
    this.feeder.setRPM(0);
    this.shooter.setShooterRPM(0, 0);
    this.shooter.setShooterAngle(26);
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