package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Feeder;

public class FeederDefault extends Command {
  private Feeder feeder;
  private TOF tof;
  private Proximity proximity;

  private boolean haveNote;
  private boolean haveCarriageNote;

  public FeederDefault(Feeder feeder, TOF tof, Proximity proximity) {
    this.feeder = feeder;
    this.tof = tof;
    this.proximity = proximity;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {
    this.haveCarriageNote = false;
    // this.haveNote = false;
    if (this.proximity.getShooterProximity()){
      this.haveNote = true;
    } else {
      this.haveNote = false;
    }
  }

  @Override
  public void execute() {
    if (this.tof.getCarriageDistMillimeters() <= Constants.SetPoints.CARRIAGE_TOF_THRESHOLD_MM){
      this.haveCarriageNote = true;
    }

    if (this.proximity.getShooterProximity()){
      this.haveNote = true;
    }

    // if (this.haveCarriageNote && !this.haveNote){
    //   this.feeder.set(120);
    // } else if (this.haveNote && !this.proximity.getShooterProximity()){
    //   this.feeder.setPercent(0);
    // } else {
    //   this.feeder.setTorque(10, 0.1);
    // }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
