// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public PresetShoot(Shooter shooter, Feeder feeder, double[] leftRightAngle /* input in RPM */  /* degrees, resting position is 20 */) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.left = leftRightAngle[0];
    this.right = leftRightAngle[1];
    this.angle = leftRightAngle[2];
    addRequirements(this.shooter, this.feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    shot = false;
    shooter.alignedPreset = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Shooter Angle", shooter.getShooterAngle());
    shooter.setShooterAngle(angle);
    shooter.setShooterRPM(left, right);
    // if (Timer.getFPGATimestamp() - startTime > 2){
    //   feeder.setPercent(0.5);
    //   intake.setPercent(0.4);
    // }
    if((Math.abs(shooter.getLeftShooterRPM() - left)/left < 0.1 && Math.abs(shooter.getShooterAngle()-angle) < 1 && Math.abs(shooter.getRightShooterRPM() + right)/right < 0.1/*   && shooter.alignedPreset */) || Timer.getFPGATimestamp() - startTime > 2) {
      feeder.setPercent(1);
      shootTime = Timer.getFPGATimestamp();
      shot = true;
    }
    SmartDashboard.putBoolean("Left RPM", Math.abs(shooter.getLeftShooterRPM() - left)/left < 0.1);
    SmartDashboard.putBoolean("Right RPM", Math.abs(shooter.getRightShooterRPM() + right)/right < 0.1);
    SmartDashboard.putBoolean("Theta Aligned", shooter.alignedPreset);
    SmartDashboard.putBoolean("Angle", Math.abs(shooter.getShooterAngle()-angle) < 2);
    SmartDashboard.putBoolean("Timeout", Timer.getFPGATimestamp() - startTime > 3);
    SmartDashboard.putBoolean("Has Shot", shot);
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
