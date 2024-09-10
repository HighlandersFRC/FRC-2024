// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class PresetShoot extends Command {
  Shooter shooter;
  Feeder feeder;
  double left;
  double right;
  double angle;
  double startTime;
  double shootTime;
  boolean shot = false;
  /** Creates a new RunShooter. */
  public PresetShoot(Shooter shooter, Feeder feeder, double left, double right, /* input in RPM */ double angle /* degrees, resting position is 20 */) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.left = left;
    this.right = right;
    this.angle = angle;
    addRequirements(this.shooter, this.feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    shot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterAngle(angle);
    shooter.setShooterRPM(left, right);
    // if (Timer.getFPGATimestamp() - startTime > 2){
    //   feeder.setPercent(0.5);
    //   intake.setPercent(0.4);
    // }
    if((Math.abs(shooter.getLeftShooterRPM() - left)/left < 0.1 && Math.abs(shooter.getShooterAngle()-angle) < 2 && Math.abs(shooter.getRightShooterRPM() + right)/right < 0.1) || Timer.getFPGATimestamp() - startTime > 3) {
      feeder.setPercent(1);
      shootTime = Timer.getFPGATimestamp();
      shot = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.setShooterAngle(20);
    this.shooter.setShooterRPM(0.0, 0.0);
    this.feeder.setPercent(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Timer.getFPGATimestamp() - shootTime > 0.2 && shot) {
    //   return true;
    // } else return false;
    return false;
  }
}
