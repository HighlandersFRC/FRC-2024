package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;

public class Shoot extends Command {
    Shooter shooter;
    double velocity;
  public Shoot(Shooter shooter, double rpm) {
    this.shooter = shooter;
    velocity = rpm;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setShooterPercent(0.701);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterPercent(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
