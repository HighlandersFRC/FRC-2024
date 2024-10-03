// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.controlloops.PID;

public class DriveThetaAligned extends Command {
  /** Creates a new DriveAutoAligned. */
  private Drive drive;
  private Peripherals peripherals;
  private Shooter shooter;
  private double turn = 0;
  private double theta;

  private PID pid;

  private double set = 0;

  private double kP = 0.055;
  private double kI = 0;
  private double kD = 0.06;

  private int angleSettled = 0;

  private double initTime = Timer.getFPGATimestamp();
  public DriveThetaAligned(Drive drive, Peripherals peripherals, Shooter shooter, double theta) {
    this.peripherals = peripherals;
    this.drive = drive;
    this.theta = theta;
    this.shooter = shooter;
    addRequirements(this.drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.



  @Override
  public void initialize() {
    pid = new PID(kP, kI, kD);
    int numTurns = ((int) peripherals.getPigeonAngle())/360;
    double pigeonAngleDegrees = peripherals.getPigeonAngle();
    double setPoint = 360 * Math.copySign(numTurns, pigeonAngleDegrees);
    // System.out.println("PA: " + pigeonAngleDegrees);
    // System.out.println("NT: " + numTurns);
    // System.out.println("A: " + setPoint);
    // System.out.println("FS: " + this.drive.getFieldSide());
    if (this.drive.getFieldSide() == "red"){
      // System.out.println("1");
      setPoint += theta;
    } else {
      // System.out.println("2");
      setPoint -= theta;
    }
    // System.out.println("B: " + setPoint);
    if (Math.abs(pigeonAngleDegrees - setPoint) > 180){
      // System.out.println("3");
      if (pigeonAngleDegrees > setPoint){
        // System.out.println("4");
        setPoint += 360;
      } else {
        // System.out.println("5");
        setPoint -= 360;
      }
    }
    // System.out.println("C: " + setPoint);
      set = setPoint;
    pid.setSetPoint(setPoint);
    pid.setMinOutput(-3);
    pid.setMaxOutput(3);
    angleSettled = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Theta Goal", theta);
    SmartDashboard.putNumber("Actual Theta", peripherals.getPigeonAngle());
    turn = peripherals.getPigeonAngle();
    pid.updatePID(turn);
    double result = -pid.getResult();
    if(Math.abs(turn - set) < 2) { 
      result = 0;
    }
    drive.driveAutoAligned(result);
    if(Math.abs(theta - peripherals.getZeroedPigeonAngleDegrees()) < 3 || Math.abs(theta - (peripherals.getZeroedPigeonAngleDegrees() - 360)) < 3 || Math.abs(theta - (peripherals.getZeroedPigeonAngleDegrees() + 360)) < 3 ) {
      shooter.alignedPreset = true;
    } else {
      shooter.alignedPreset = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.alignedPreset = true;
    // drive.autoDrive(new Vector(0, 0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(!OI.driverController.getXButton()) {
    //   return true;
    // }
    return false;
  }
}