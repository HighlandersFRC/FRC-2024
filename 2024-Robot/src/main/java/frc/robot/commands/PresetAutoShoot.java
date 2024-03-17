// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

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

public class PresetAutoShoot extends Command {
  private Drive drive;
  private Shooter shooter;
  private Feeder feeder;
  private Peripherals peripherals;
  private Lights lights;
  private Proximity proximity;

  private double shooterDegrees;
  private double shooterRPM;
  private double feederRPM;

  private double startTime;
  private double timeout = 1;

  private double shotTime = 0;
  private boolean hasShot;
  private double shotPauseTime = 0.0;

  private double shooterDegreesAllowedError = 0.75;
  private double shooterRPMAllowedError = 200;
  private double driveAngleAllowedError = 2;

  private double lookAheadTime = 0.0;

  private boolean hasReachedSetPoint;

  private PID pid;

  private double kP = 0.04;
  private double kI = 0;
  private double kD = 0.06;

  private double speakerAngleDegrees;

  private double robotAngleOffset;

  private boolean smartDashboardTuning;

  public PresetAutoShoot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, Proximity proximity, double shooterDegrees, double shooterRPM, double feederRPM, double robotAngleOffset) {
    this.drive = drive;
    this.shooter = shooter;
    this.feeder = feeder;
    this.peripherals = peripherals;
    this.lights = lights;
    this.proximity = proximity;
    this.shooterDegrees = shooterDegrees;
    this.shooterRPM = shooterRPM;
    this.feederRPM = feederRPM;
    this.robotAngleOffset = robotAngleOffset;
    if (this.drive.getFieldSide() == "blue"){
      this.robotAngleOffset *= -1;
    }
    addRequirements(this.drive, this.shooter, this.feeder);
  }

  public PresetAutoShoot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, Proximity proximity, double shooterDegrees, double shooterRPM, double feederRPM, double robotAngleOffset, double timeout) {
    this.drive = drive;
    this.shooter = shooter;
    this.feeder = feeder;
    this.peripherals = peripherals;
    this.lights = lights;
    this.proximity = proximity;
    this.shooterDegrees = shooterDegrees;
    this.shooterRPM = shooterRPM;
    this.feederRPM = feederRPM;
    this.robotAngleOffset = robotAngleOffset;
    if (this.drive.getFieldSide() == "blue"){
      this.robotAngleOffset *= -1;
    }
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
    lights.setCommandRunning(true);
    lights.setStrobePurple();
    this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
  }

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
      lights.setStrobeGreen();
      this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
    }

    this.pid.setSetPoint(pigeonAngleDegrees - this.speakerAngleDegrees + this.robotAngleOffset);
    this.pid.updatePID(pigeonAngleDegrees);
    double turnResult = -pid.getResult();    

    if (canSeeTag && this.speakerAngleDegrees < 90){
      this.drive.driveAutoAligned(turnResult);
    } else {
      this.drive.driveAutoAligned(0);
    }

    this.shooter.set(this.shooterDegrees, this.shooterRPM);

    if (Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(this.shooter.getFlywheelRPM() - this.shooterRPM) <= this.shooterRPMAllowedError && Math.abs(this.speakerAngleDegrees - this.robotAngleOffset) <= this.driveAngleAllowedError){
      lights.clearAnimations();
      lights.setCandleRGB(0, 255, 0);
      this.hasReachedSetPoint = true;
    }

    if (this.hasReachedSetPoint == true){
      this.feeder.set(this.feederRPM);
    } else {
      this.feeder.set(0.0);
    }

    if (!this.proximity.getFeederProximity() && !this.hasShot){
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
    // System.out.println("Elev Err: " + Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees));
    // System.out.println("Pigeon Angle: " + pigeonAngleDegrees);
    // System.out.println("Targ. Pigeon Angle: " + (pigeonAngleDegrees - this.speakerAngleDegrees));
    // System.out.println("Pigeon Angle Err: " + Math.abs(this.speakerAngleDegrees));
    // System.out.println("<================>");
  }

  @Override
  public void end(boolean interrupted) {
    this.feeder.set(0);
    lights.clearAnimations();
    lights.setCommandRunning(false);
  }

  @Override
  public boolean isFinished() {
    if (this.shooterDegrees > 90){
      return true;
    } else if (this.hasShot && Timer.getFPGATimestamp() - this.shotTime > this.shotPauseTime){
      return true;
    } else {
      return false;
    }
  }
}
