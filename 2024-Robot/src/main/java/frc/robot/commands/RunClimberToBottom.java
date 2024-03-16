package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class RunClimberToBottom extends Command {
  private Climber climber;
  private double current;
  private double maxPercent;

  public RunClimberToBottom(Climber climber, double current, double maxPercent) {
    this.climber = climber;
    this.current = current;
    this.maxPercent = maxPercent;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.climber.setTrapRollerPercent(0);
    this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kFEED);
    // System.out.println("armfeed2");
    this.climber.setElevatorTorque(this.current, this.maxPercent);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
