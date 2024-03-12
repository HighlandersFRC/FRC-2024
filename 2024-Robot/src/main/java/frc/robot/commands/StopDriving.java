package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.tools.math.Vector;

public class StopDriving extends Command {
  private Drive drive;

  public StopDriving(Drive drive) {
    this.drive = drive;
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.drive.autoDrive(new Vector(0, 0), 0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
