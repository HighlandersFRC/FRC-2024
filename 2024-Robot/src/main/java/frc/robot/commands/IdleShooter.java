package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class IdleShooter extends Command {
  private Shooter shooter;
  private double RPM;

  public IdleShooter(Shooter shooter, double RPM) {
    this.shooter = shooter;
    this.RPM = RPM;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.shooter.setVelocity(this.RPM);
    if (Math.abs(this.shooter.getAngleDegrees() - Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG) < 2){
      this.shooter.setAnglePercent(0);
    } else {
      this.shooter.setAnglePercent(-0.05);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
