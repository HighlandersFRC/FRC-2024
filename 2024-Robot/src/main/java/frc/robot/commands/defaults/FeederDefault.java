package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Feeder;

public class FeederDefault extends Command {
  private Feeder feeder;
  private TOF tof;

  private boolean haveNote;
  private boolean haveCarriageNote;

  public FeederDefault(Feeder feeder, TOF tof) {
    this.feeder = feeder;
    this.tof = tof;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {
    this.haveCarriageNote = false;
    this.haveNote = false;
  }

  @Override
  public void execute() {
    if (this.tof.getCarriageDistMillimeters() <= Constants.SetPoints.CARRIAGE_TOF_THRESHOLD_MM){
      this.haveCarriageNote = true;
    }

    if (this.tof.getFeederDistMillimeters() <= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM){
      this.haveNote = true;
    }

    if (this.haveCarriageNote && !this.haveNote){
      this.feeder.set(120);
    } else if (this.haveNote){
      this.feeder.setPercent(0);
    } else {
      this.feeder.setPercent(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
