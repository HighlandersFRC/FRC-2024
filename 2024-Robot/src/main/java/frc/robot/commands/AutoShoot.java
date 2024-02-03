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
  private double shotPauseTime = 0.1;

  private double shooterDegreesAllowedError = 1;
  private double shooterRPMAllowedError = 100;
  private double driveAngleAllowedError = 1;

  private double angle;

  private boolean hasReachedSetPoint;

  private PID pid;

  private double set = 0;

  private double kP = 0.07;
  private double kI = 0;
  private double kD = 0.02;

  private double turn;
  private double targetPigeonAngle;

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
    this.angle = peripherals.getFrontCamTargetTy();
    this.hasReachedSetPoint = false;
    pid = new PID(kP, kI, kD);
    pid.setMinOutput(-3);
    pid.setMaxOutput(3);

    turn = peripherals.getFrontCamTargetTx();
    double pigeonAngle = peripherals.getPigeonAngle();
    this.targetPigeonAngle = pigeonAngle - turn;
    pid.setSetPoint(targetPigeonAngle);

    // this.feeder.set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turn = peripherals.getFrontCamTargetTx();
    double pigeonAngle = peripherals.getPigeonAngle();
    pid.updatePID(pigeonAngle);

    double result = -pid.getResult();

    ArrayList<Integer> ids = peripherals.getFrontCamIDs();

    boolean canSeeTag = false;
    for (int id : ids){
      if (id == 7 || id == 4){
        canSeeTag = true;
      }
    }

    if (canSeeTag && turn < 90){
      this.drive.driveAutoAligned(result);
    } else {
      this.drive.autoRobotCentricTurn(0);
    }

    if (canSeeTag){
      this.angle = peripherals.getFrontCamTargetTy();
      this.shooterValues = Constants.SetPoints.getShooterValues(angle);
      this.shooterDegrees = shooterValues[0];
      this.shooterRPM = shooterValues[1];
    } 

    this.shooter.set(this.shooterDegrees, this.shooterRPM);

    if (Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(this.shooter.getFlywheelRPM() - this.shooterRPM) <= this.shooterRPMAllowedError && Math.abs(pigeonAngle - this.targetPigeonAngle) <= this.driveAngleAllowedError){
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

    System.out.println("RPM: " + this.shooter.getFlywheelRPM());
    System.out.println("RPM Err: " + Math.abs(this.shooter.getFlywheelRPM() - this.shooterRPM));
    System.out.println("Angle: " + this.shooter.getAngleDegrees());
    System.out.println("Angle Err: " + Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees));
    System.out.println("Pigeon Angle: " + pigeonAngle);
    System.out.println("Pigeon Angle Err: " + Math.abs(pigeonAngle - targetPigeonAngle));
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
