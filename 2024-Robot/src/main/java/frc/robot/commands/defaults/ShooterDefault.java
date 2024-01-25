package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterDefault extends Command {
  Shooter shooter;

  public ShooterDefault(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // System.out.println("Shooter Default");
    this.shooter.setShooterPercent(0);
    this.shooter.setShooterAngle(30);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
