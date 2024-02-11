// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

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
  // private double timeout = 4;
  private double timeout = 20;

  private double shotTime = 0;
  private boolean hasShot;
  private double shotPauseTime = 0.0;

  private double shooterDegreesAllowedError = 1;
  private double shooterRPMAllowedError = 400;
  private double driveAngleAllowedError = 3;

  private double lookAheadTime = 0.0;

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
      this.speakerElevationDegrees = this.peripherals.getFrontCamTargetTy();
      this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
      this.shooterValues = Constants.SetPoints.getShooterValuesFromAngle(speakerElevationDegrees);
      this.shooterDegrees = shooterValues[0];
      this.shooterRPM = shooterValues[1];
    }

    //velocity compensation
    double[] currentSetpoints = Constants.SetPoints.getVelocityAdjustedSetpoint(pigeonAngleDegrees, this.speakerAngleDegrees, this.shooterDegrees, this.shooterRPM, this.drive.getRobotVelocityVector());
    double targetCurrentPigeonAngleDegrees = currentSetpoints[0];
    double targetCurrentShooterDegrees = currentSetpoints[1];
    double targetCurrentShooterRPM = currentSetpoints[2];

    double currentRobotSpeakerAngleDegrees = pigeonAngleDegrees - this.speakerAngleDegrees;
    double currentSpeakerDistance = Constants.SetPoints.getDistFromAngle(this.speakerElevationDegrees);
    // System.out.println("Current Dist: " + currentSpeakerDistance);
    Vector currentVelocityVector = this.drive.getRobotVelocityVector();
    Vector currentAccelerationVector = this.drive.getRobotAccelerationVector();
    // System.out.println("Accel: (" + currentAccelerationVector.getI() + ", " + currentAccelerationVector.getJ() + ")");
    Vector futureVelocityVector = new Vector();
    futureVelocityVector.setI(currentVelocityVector.getI() + currentAccelerationVector.getI() * this.lookAheadTime);
    futureVelocityVector.setJ(currentVelocityVector.getJ() + currentAccelerationVector.getJ() * this.lookAheadTime);
    // System.out.println("Future Vel: (" + futureVelocityVector.getI() + ", " + futureVelocityVector.getJ() + ")");
    double deltaX = 0.5 * currentAccelerationVector.getI() * Math.pow(this.lookAheadTime, 2) + currentVelocityVector.getI() * this.lookAheadTime;
    double deltaY = 0.5 * currentAccelerationVector.getJ() * Math.pow(this.lookAheadTime, 2) + currentVelocityVector.getJ() * this.lookAheadTime;
    // System.out.println("Delta X: " + deltaX);
    // System.out.println("Delta Y: " + deltaY);
    double deltaAngleRadians = Math.atan2(deltaY, deltaX);
    // System.out.println("Delta Angle Rad: " + deltaAngleRadians);
    double deltaD = -Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)) * Math.cos(deltaAngleRadians - Math.toRadians(currentRobotSpeakerAngleDegrees));
    // System.out.println("Delta D: " + deltaD);
    double futureSpeakerDistance = currentSpeakerDistance + deltaD;
    double[] futureSetpoints = Constants.SetPoints.getShooterValuesFromDistance(futureSpeakerDistance);
    // System.out.println("Future Degrees: " + futureSetpoints[0]);
    // System.out.println("Future RPM: " + futureSetpoints[1]);
    double[] futureAdjutedSetpoints = Constants.SetPoints.getVelocityAdjustedSetpoint(pigeonAngleDegrees, this.speakerAngleDegrees, futureSetpoints[0], futureSetpoints[1], futureVelocityVector);
    double targetFuturePigeonAngleDegrees = futureAdjutedSetpoints[0];
    double targetFutureShooterDegrees = futureAdjutedSetpoints[1];
    double targetFutureShooterRPM = futureAdjutedSetpoints[2];

    // System.out.println("Future Adj. Degrees: " + targetFutureShooterDegrees);
    // System.out.println("Future Adj. RPM: " + targetFutureShooterRPM);

    this.pid.setSetPoint(targetCurrentPigeonAngleDegrees);
    this.pid.updatePID(pigeonAngleDegrees);
    double turnResult = -pid.getResult();    

    if (canSeeTag && this.speakerAngleDegrees < 90){
      this.drive.driveAutoAligned(turnResult);
    } else {
      this.drive.driveAutoAligned(0);
    }

    this.shooter.set(targetFutureShooterDegrees, targetFutureShooterRPM);

    if (Math.abs(this.shooter.getAngleDegrees() - targetCurrentShooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(this.shooter.getFlywheelRPM() - targetCurrentShooterRPM) <= this.shooterRPMAllowedError && Math.abs(pigeonAngleDegrees - targetCurrentPigeonAngleDegrees) <= this.driveAngleAllowedError){
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

    if (Timer.getFPGATimestamp() - this.startTime >= this.timeout){
      this.hasReachedSetPoint = true;
    }

    // System.out.println("RPM: " + this.shooter.getFlywheelRPM());
    // System.out.println("Targ. RPM: " + targetShooterRPM);
    // System.out.println("RPM Err: " + Math.abs(this.shooter.getFlywheelRPM() - targetShooterRPM));
    // System.out.println("Elev: " + this.shooter.getAngleDegrees());
    // System.out.println("Targ. Elev: " + targetShooterDegrees);
    // System.out.println("Elev Err: " + Math.abs(this.shooter.getAngleDegrees() - targetShooterDegrees));
    // System.out.println("Pigeon Angle: " + pigeonAngleDegrees);
    // System.out.println("Targ. Pigeon Angle: " + targetCurrentPigeonAngleDegrees);
    // System.out.println("Pigeon Angle Err: " + Math.abs(pigeonAngleDegrees - targetCurrentPigeonAngleDegrees));
    // System.out.println("<================>");

    SmartDashboard.putNumber("Targ RPM", targetCurrentShooterRPM);
    SmartDashboard.putNumber("Targ Degrees", targetCurrentShooterDegrees);
    SmartDashboard.putNumber("Targ Pigeon Angle", targetCurrentPigeonAngleDegrees);
    SmartDashboard.putNumber("RPM", this.shooter.getFlywheelRPM());
    SmartDashboard.putNumber("Degrees", this.shooter.getAngleDegrees());
    SmartDashboard.putNumber("Pigeon Angle", pigeonAngleDegrees);
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
    } else if (this.hasShot && Timer.getFPGATimestamp() - this.shotTime >= this.shotPauseTime){
      return true;
    } else {
      return false;
    }
  }
}
