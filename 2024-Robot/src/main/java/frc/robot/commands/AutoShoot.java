// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
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
  private double timeout = 10;

  private double shotTime = 0;
  private boolean hasShot;
  private double shotPauseTime = 0.0;

  // private double shooterDegreesAllowedError = 1;
  // private double shooterRPMAllowedError = 100;
  // private double driveAngleAllowedError = 2;
  private double shooterDegreesAllowedError = 2;
  private double shooterRPMAllowedError = 500;
  private double driveAngleAllowedError = 4;

  private double angle;

  private boolean hasReachedSetPoint;

  private PID pid;

  private double kP = 0.04;
  private double kI = 0;
  private double kD = 0.06;

  private double speakerElevationDegrees;
  private double speakerAngleDegrees;

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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
    this.hasShot = false;
    this.hasReachedSetPoint = false;
    pid = new PID(kP, kI, kD);
    pid.setMinOutput(-3);
    pid.setMaxOutput(3);

    this.speakerElevationDegrees = peripherals.getFrontCamTargetTy();
    this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pigeonAngleDegrees = this.peripherals.getPigeonAngle();

    ArrayList<Integer> ids = peripherals.getFrontCamIDs();

    boolean canSeeTag = false;
    for (int id : ids){
      if (id == 7 || id == 4){
        canSeeTag = true;
      }
    }

    if (canSeeTag){
      this.speakerElevationDegrees = peripherals.getFrontCamTargetTy();
      this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
      this.shooterValues = Constants.SetPoints.getShooterValues(speakerElevationDegrees);
      this.shooterDegrees = shooterValues[0];
      this.shooterRPM = shooterValues[1];
    }

    //velocity compensation
    double thetaI = Math.toRadians(pigeonAngleDegrees - this.speakerAngleDegrees);
    double phiI = Math.toRadians(this.shooterDegrees);
    double rhoI = Constants.Physical.flywheelRPMToNoteMPS(this.shooterRPM);
    Vector robotVelocityVector = this.drive.getRobotVelocityVector();
    double vx = robotVelocityVector.getI();
    double vy = robotVelocityVector.getJ();

    double xI = rhoI * Math.cos(phiI) * Math.cos(thetaI);
    double yI = rhoI * Math.cos(phiI) * Math.sin(thetaI);
    double zI = rhoI * Math.sin(phiI);

    double xF = xI - vx;
    double yF = yI - vy;
    double zF = zI;

    double thetaF = Math.atan2(yF, xF);
    if (thetaF < 0){
      thetaF += 2 * Math.PI;
    }
    double phiF = Math.atan2(zF, Math.sqrt(Math.pow(xF, 2) + Math.pow(yF, 2)));
    double rhoF = Math.sqrt(Math.pow(xF, 2) + Math.pow(yF, 2) + Math.pow(zF, 2));
    // System.out.println("ThetaI: " + thetaI);
    // System.out.println("PhiI: " + phiI);
    // System.out.println("RhoI: " + rhoI);
    // System.out.println("ThetaF: " + thetaF);
    // System.out.println("PhiF: " + phiF);
    // System.out.println("RhoF: " + rhoF);

    double targetShooterDegrees = Math.toDegrees(phiF);
    double targetShooterRPM = Constants.Physical.noteMPSToFlywheelRPM(rhoF);

    double extraPigeonRotations = Math.floor((pigeonAngleDegrees / 360.0));
    // System.out.println("Extra Rotations: " + extraPigeonRotations);
    double targetPigeonAngleDegrees = extraPigeonRotations * 360.0 + Math.toDegrees(thetaF);

    this.pid.setSetPoint(targetPigeonAngleDegrees);
    this.pid.updatePID(pigeonAngleDegrees);
    double turnResult = -pid.getResult();    

    if (canSeeTag && this.speakerAngleDegrees < 90){
      this.drive.driveAutoAligned(turnResult);
    } else {
      this.drive.driveAutoAligned(0);
    }

    this.shooter.set(targetShooterDegrees, targetShooterRPM);

    if (Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(this.shooter.getFlywheelRPM() - this.shooterRPM) <= this.shooterRPMAllowedError && Math.abs(pigeonAngleDegrees - targetPigeonAngleDegrees) <= this.driveAngleAllowedError){
      this.hasReachedSetPoint = true;
    }

    if (this.hasReachedSetPoint == true){
      this.feeder.set(this.feederRPM);
    } else {
      this.feeder.set(0.0);
    }

    if (this.tof.getFeederDistMillimeters() >= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM && !this.hasShot){
      this.hasShot = true;
      this.shotTime = Timer.getFPGATimestamp();
    }

    // System.out.println("RPM: " + this.shooter.getFlywheelRPM());
    // System.out.println("Targ. RPM: " + targetShooterRPM);
    // System.out.println("RPM Err: " + Math.abs(this.shooter.getFlywheelRPM() - targetShooterRPM));
    // System.out.println("Elev: " + this.shooter.getAngleDegrees());
    // System.out.println("Targ. Elev: " + targetShooterDegrees);
    // System.out.println("Elev Err: " + Math.abs(this.shooter.getAngleDegrees() - targetShooterDegrees));
    // System.out.println("Pigeon Angle: " + pigeonAngleDegrees);
    // System.out.println("Targ. Pigeon Angle: " + targetPigeonAngleDegrees);
    // System.out.println("Pigeon Angle Err: " + Math.abs(pigeonAngleDegrees - targetPigeonAngleDegrees));
    // System.out.println("<================>");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.feeder.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("Duration " + (Timer.getFPGATimestamp() - this.startTime));
    // System.out.println("Has Shot " + this.hasShot);
    // System.out.println("Time Since Shot " + (Timer.getFPGATimestamp() - this.shotTime));

    if (this.shooterDegrees > 90){
      return true;
    }
    if (Timer.getFPGATimestamp() - this.startTime >= this.timeout){
      System.out.println("Shooter time out");
      return true;
    } else if (this.hasShot && Timer.getFPGATimestamp() - this.shotTime >= this.shotPauseTime){
      return true;
    } else {
      return false;
    }
  }
}
