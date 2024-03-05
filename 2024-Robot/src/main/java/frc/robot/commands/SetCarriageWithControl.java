package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Climber;

public class SetCarriageWithControl extends Command {
  private Climber climber;
  private double degrees;
  private double current;
  private double maxPercent;
  private boolean timeout;
  private double initTime;

  public SetCarriageWithControl(Climber climber, double degrees, double current, double maxPercent, boolean timeout) {
    this.climber = climber;
    this.degrees = degrees;
    this.current = current;
    this.maxPercent = maxPercent;
    this.timeout = timeout;
    addRequirements(this.climber);
  }

  public SetCarriageWithControl(Climber climber, Constants.SetPoints.CarriageRotation carriageRotation, double current, double maxPercent, boolean timeout) {
    this.climber = climber;
    this.degrees = carriageRotation.degrees;
    this.current = current;
    this.maxPercent = maxPercent;
    this.timeout = timeout;
    addRequirements(this.climber);
  }

  @Override
  public void initialize() {
    this.initTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    this.climber.setCarriageRotationDegrees(this.degrees);
    if (OI.getDriverRB()){
      this.climber.setTrapRollerTorque(20, 0.5);
    } else {
      this.climber.setTrapRollerTorque(this.current, this.maxPercent);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (this.timeout && Timer.getFPGATimestamp() - this.initTime > 2){
      return true;
    } else {
      return false;
    }
  }
}
