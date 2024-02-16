package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  Shooter shooter;
  double angle;
  double RPM;
  double startTime = Timer.getFPGATimestamp();

  public RunShooter(Shooter shooter, double angle, double RPM) {
    this.shooter = shooter;
    this.angle = angle;
    this.RPM = RPM;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    this.shooter.set(this.angle, this.RPM);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
