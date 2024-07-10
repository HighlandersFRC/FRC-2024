// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class DipShot extends Command {
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
  private double shooterRPMAllowedError = 100;
  private double driveAngleAllowedError = 2;

  private double lookAheadTime = 0.0;

  private boolean hasReachedSetPoint;

  private PID pid;

  private double kP = 0.045;
  private double kI = 0;
  private double kD = 0.065;

  private double speakerAngleDegrees;

  private double targetPigeonAngleDegrees;

  private double robotAngleOffset;
  private double redAngle;
  private double blueAngle;

  public DipShot(Drive drive, Shooter shooter, Feeder feeder, Peripherals peripherals, Lights lights, Proximity proximity, double shooterDegrees, double shooterRPM, double feederRPM, double robotAngleOffset, double redAngle, double blueAngle, double timeout) {
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
    this.redAngle = redAngle;
    this.blueAngle = blueAngle;
    this.timeout = timeout;
    addRequirements(this.shooter, this.feeder);
  }

  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
    this.hasShot = false;
    this.hasReachedSetPoint = false;
    pid = new PID(kP, kI, kD);
    pid.setMinOutput(-3);
    pid.setMaxOutput(3);

    this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();

    if (OI.isRedSide()){
      // System.out.println("RED");
        this.targetPigeonAngleDegrees = this.redAngle;
    } else {
      // System.out.println("BLUE");
        this.targetPigeonAngleDegrees = this.blueAngle;
    }
    double standardizedPigeonAngleDegrees = Constants.SetPoints.standardizeAngleDegrees(this.peripherals.getPigeonAngle());
    double dif = standardizedPigeonAngleDegrees - this.targetPigeonAngleDegrees;
    if (Math.abs(dif) > 180){
        this.targetPigeonAngleDegrees -= 360;
    }
    
  }

  @Override
  public void execute() {
    double pigeonAngleDegrees = this.peripherals.getPigeonAngle();

    

    this.pid.setSetPoint(this.targetPigeonAngleDegrees);
    this.pid.updatePID(Constants.SetPoints.standardizeAngleDegrees(pigeonAngleDegrees));
    // System.out.println("Target: " + this.targetPigeonAngleDegrees);
    // System.out.println("Current: " + Constants.SetPoints.standardizeAngleDegrees(pigeonAngleDegrees));
    double turnResult = -pid.getResult();    

    // this.drive.driveAutoAligned(turnResult);

    this.shooter.set(this.shooterDegrees, this.shooterRPM);

    if (Math.abs(this.shooter.getAngleDegrees() - this.shooterDegrees) <= this.shooterDegreesAllowedError && Math.abs(this.shooter.getFlywheelRPM() - this.shooterRPM) <= this.shooterRPMAllowedError){
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
      // System.out.println("Lob Timeout");
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
  }

  @Override
  public boolean isFinished() {
    if (this.shooterDegrees > 90){
      return true;
    } else if (this.hasShot && Timer.getFPGATimestamp() - this.shotTime > this.shotPauseTime){
      return true;
    } else if (OI.getPOV() == -1){
      return true;
    } else {
      return false;
    }
  }
}