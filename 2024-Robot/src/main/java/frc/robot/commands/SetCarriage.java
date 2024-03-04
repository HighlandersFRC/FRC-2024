package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class SetCarriage extends Command {
  private Climber climber;
  private double degrees;
  private double Current;
  private double maxPercent;

  public SetCarriage(Climber climber, double degrees, double Current, double maxPercent) {
    this.climber = climber;
    this.degrees = degrees;
    this.Current = Current;
    this.maxPercent = maxPercent;
    addRequirements(this.climber);
  }

  public SetCarriage(Climber climber, Constants.SetPoints.CarriageRotation carriageRotation, double Current, double maxPercent) {
    this.climber = climber;
    this.degrees = carriageRotation.degrees;
    this.Current = Current;
    this.maxPercent = maxPercent;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.climber.setCarriageRotationDegrees(this.degrees);
    this.climber.setTrapRollerTorque(this.Current, this.maxPercent);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
