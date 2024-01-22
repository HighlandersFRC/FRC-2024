package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;

public class Shoot extends Command {
    Shooter shooter;
    Feeder feeder;
    double velocity;
  public Shoot(Shooter shooter, Feeder feeder, double rpm) {
    this.shooter = shooter;
    this.feeder = feeder;
    velocity = rpm;
    addRequirements(shooter, feeder);
  }

  @Override
  public void initialize() {
    shooter.setShooterVelocity(velocity);
    feeder.setFeeder(velocity);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterVelocity(0);
    feeder.setFeeder(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
