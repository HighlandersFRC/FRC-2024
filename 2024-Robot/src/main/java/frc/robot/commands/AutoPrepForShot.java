package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Shooter;

public class AutoPrepForShot extends Command {
  private Shooter shooter;
  private TOF tof;
  private double predictedShooterDegrees;
  private double predictedShooterRPM;
  
  private boolean haveNote;

  public AutoPrepForShot(Shooter shooter, TOF tof, double predictedShooterDegrees, double predictedShooterRPM) {
    this.shooter = shooter;
    this.tof = tof;
    this.predictedShooterDegrees = predictedShooterDegrees;
    this.predictedShooterRPM = predictedShooterRPM;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    this.haveNote = false;
  }

  @Override
  public void execute() {
    if (this.tof.getFeederDistMillimeters() <= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM){
      this.haveNote = true;
    }

    if (this.haveNote){
      this.shooter.set(this.predictedShooterDegrees, this.predictedShooterRPM);
    } else {
      if (Math.abs(this.shooter.getAngleDegrees() - Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG) < 2){
        this.shooter.setAnglePercent(0);
      } else {
        this.shooter.setAnglePercent(-0.05);
      }
      this.shooter.setFlywheelRPM(this.predictedShooterRPM);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
