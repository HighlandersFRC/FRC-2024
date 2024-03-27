package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Shooter;

public class AutoPrepForShot extends Command {
  private Shooter shooter;
  private Proximity proximity;
  private double predictedShooterDegrees;
  private double predictedShooterRPM;
  
  private boolean haveNote;

  public AutoPrepForShot(Shooter shooter, Proximity proximity, double predictedShooterDegrees, double predictedShooterRPM) {
    this.shooter = shooter;
    this.proximity = proximity;
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
    if (this.proximity.getFeederProximity() && !this.proximity.getShooterProximity()){
      this.haveNote = true;
    }

    if (haveNote){
      this.shooter.set(predictedShooterDegrees, predictedShooterRPM);
    } else if (Math.abs(this.shooter.getAngleDegrees() - Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG) < 2){
      this.shooter.setAnglePercent(0);
    } else {
      this.shooter.setAngleTorque(-10, 0.4);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
