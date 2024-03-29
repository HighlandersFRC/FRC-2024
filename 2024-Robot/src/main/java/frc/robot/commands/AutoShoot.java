package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.TOF;
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
  private TOF tof;

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
  private double driveAngleAllowedError = 1;

  private double lookAheadTime = 0.0;

  private boolean hasReachedSetPoint;

  private PID pid;

  private double kP = 0.05;
  private double kI = 0;
  private double kD = 0.06;

  private double speakerElevationDegrees;
  private double speakerAngleDegrees;
  private double targetPigeonAngleDegrees;

  public AutoShoot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, TOF tof, double feederRPM) {
    this.drive = drive;
    this.shooter = shooter;
    this.feeder = feeder;
    this.peripherals = peripherals;
    this.lights = lights;
    this.tof = tof;
    this.feederRPM = feederRPM;
    addRequirements(this.drive, this.shooter, this.feeder);
  }

  public AutoShoot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, TOF tof, double feederRPM, double timeout) {
    this.drive = drive;
    this.shooter = shooter;
    this.feeder = feeder;
    this.peripherals = peripherals;
    this.lights = lights;
    this.tof = tof;
    this.feederRPM = feederRPM;
    this.timeout = timeout;
    addRequirements(this.drive, this.shooter, this.feeder);
  }

  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
    this.hasShot = false;
    this.hasReachedSetPoint = false;
    pid = new PID(kP, kI, kD);
    pid.setMinOutput(-3);
    pid.setMaxOutput(3);
    this.reachedSetPointTime = 0;
    this.speakerElevationDegrees = 0;
    this.speakerAngleDegrees = 0;
    this.shooterValues = Constants.SetPoints.getShooterValuesFromAngle(this.speakerElevationDegrees);
    this.shooterDegrees = this.shooterValues[0];
    this.shooterRPM = this.shooterValues[1];
  }

  @Override
  public void execute() {

    double pigeonAngleDegrees = this.peripherals.getPigeonAngle();

    ArrayList<Integer> ids = this.peripherals.getFrontCamIDs();

    canSeeTag = false;
    for (int id : ids){
      if (id == 7 || id == 4){
        canSeeTag = true;
      }
    }

    if (canSeeTag){
      this.speakerElevationDegrees = this.peripherals.getFrontCamTargetTy();
      this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
      if (this.speakerElevationDegrees < 90 && this.speakerAngleDegrees < 90){
        this.shooterValues = Constants.SetPoints.getShooterValuesFromAngle(this.speakerElevationDegrees);
        this.shooterDegrees = this.shooterValues[0];
        this.shooterRPM = this.shooterValues[1];
        this.driveAngleAllowedError = Constants.SetPoints.getAllowedAngleErrFromAngle(this.speakerElevationDegrees);
      }
    }

    if (canSeeTag && this.speakerAngleDegrees < 90){
      double initialTargetPigeonAngleDegrees = pigeonAngleDegrees - this.speakerAngleDegrees;
      // System.out.println("Initial Degrees: " + initialTargetPigeonAngleDegrees);
      // this.targetPigeonAngleDegrees = Constants.SetPoints.getAdjustedPigeonAngle(initialTargetPigeonAngleDegrees, Constants.SetPoints.getDistFromAngle(this.speakerElevationDegrees));
      this.targetPigeonAngleDegrees = initialTargetPigeonAngleDegrees;
      // System.out.println("Adjusted Degrees: " + this.targetPigeonAngleDegrees);
    }

    this.pid.setSetPoint(this.targetPigeonAngleDegrees);
    this.pid.updatePID(pigeonAngleDegrees);
    double turnResult = -pid.getResult();

    if (Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(this.shooter.getFlywheelRPM() - this.shooterRPM) <= this.shooterRPMAllowedError && Math.abs(pigeonAngleDegrees - this.targetPigeonAngleDegrees) <= this.driveAngleAllowedError){
      this.hasReachedSetPoint = true;
      this.reachedSetPointTime = Timer.getFPGATimestamp();
      turnResult = 0;
    }

    if (canSeeTag && this.speakerAngleDegrees < 90){
      this.drive.driveAutoAligned(turnResult);
    } else {
      this.drive.driveAutoAligned(0);
    }

    this.shooter.set(this.shooterDegrees, this.shooterRPM);

    if (this.hasReachedSetPoint == true){
      this.feeder.set(this.feederRPM);
    } else {
      this.feeder.set(0.0);
    }

    if (this.tof.getFeederDistMillimeters() >= 110.0 && !this.hasShot){
      this.hasShot = true;
      this.shotTime = Timer.getFPGATimestamp();
    }

    if (Timer.getFPGATimestamp() - this.startTime >= this.timeout){
      this.hasReachedSetPoint = true;
    }

    // System.out.println("RPM: " + this.shooter.getFlywheelRPM());
    // System.out.println("Targ. RPM: " + this.shooterRPM);
    // System.out.println("RPM Err: " + Math.abs(this.shooter.getFlywheelRPM() - this.shooterRPM));
    // System.out.println("Elev: " + this.shooter.getAngleDegrees());
    // System.out.println("Targ. Elev: " + this.shooterDegrees);
    // System.out.println("Elev Err: " + Math.abs(this.shooter.getAngleDegrees() - shooterDegrees));
    // System.out.println("Pigeon Angle: " + pigeonAngleDegrees);
    // System.out.println("Targ. Pigeon Angle: " + this.targetPigeonAngleDegrees);
    // System.out.println("Pigeon Angle Err: " + Math.abs(pigeonAngleDegrees - this.targetPigeonAngleDegrees));
    // System.out.println("Turn Result: " + turnResult);
    // System.out.println("Speaker Ang Deg: " + this.speakerAngleDegrees);
    // System.out.println("Speaker Elev Deg: " + this.speakerElevationDegrees);
    // System.out.println("<================>");
  }

  @Override
  public void end(boolean interrupted) {
    this.feeder.set(0);
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
    } else {
      return false;
    }
  }
}