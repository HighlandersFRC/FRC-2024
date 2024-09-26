package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

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

public class PositionalSpinUp extends Command {
  public static boolean canSeeTag;
  private Drive drive;
  private Shooter shooter;
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

  private double defaultShooterAngle = 0;
  private double defaultFlywheelRPM = 0;

  private ArrayList<double[]> tagReadings = new ArrayList<double[]>();
  private ArrayList<Double> pigeonAngles = new ArrayList<Double>();

  private boolean haveGoodSetpoint = false;
  private boolean auto = false;
  private int numTimesNoTrack = 0;

  public PositionalSpinUp(Drive drive, Shooter shooter, Peripherals peripherals, Lights lights, Proximity proximity) {
    this.shooter = shooter;
    this.drive = drive;
    this.peripherals = peripherals;
    this.lights = lights;
    this.proximity = proximity;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
    this.pigeonAngles = new ArrayList<Double>();
    this.hasShot = false;
    this.hasReachedSetPoint = false;
    this.haveGoodSetpoint = false;
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
  }

  @Override
  public void execute() {
    this.distToSpeakerMeters = Constants.getDistance(Constants.Physical.SPEAKER_X, Constants.Physical.SPEAKER_Y, drive.getMT2OdometryX(), drive.getMT2OdometryY());
    this.angleToSpeakerDegrees = Constants.getAngleToPoint(Constants.Physical.SPEAKER_X + (Constants.Physical.SPEAKER_DEPTH / 2), Constants.Physical.SPEAKER_Y + 0.07, drive.getMT2OdometryX(), drive.getMT2OdometryY());
    this.shooterValues = Constants.SetPoints.getShooterValuesFromDistance(this.distToSpeakerMeters, false);
    this.shooterDegrees = this.shooterValues[0];
    this.shooterRPM = this.shooterValues[1];
    
    // System.out.println("dist: " + distToSpeakerMeters);
    // System.out.println("angle: " + angleToSpeakerDegrees);
    // System.out.println("deg" + shooterDegrees);
    // System.out.println("rpm" + shooterRPM);
    Logger.recordOutput("spinupshooterangle", shooterDegrees);
    Logger.recordOutput("spinupshooterrpm", shooterRPM);
    if (this.proximity.getFeederProximity()){
      this.shooter.set(this.shooterDegrees, this.shooterRPM);
    } else {
      this.shooter.setFlywheelRPM(this.shooterRPM);
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
    }else {
      return false;
    }
  }
}