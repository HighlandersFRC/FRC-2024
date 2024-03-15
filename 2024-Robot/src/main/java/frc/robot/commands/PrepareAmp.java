package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;

public class PrepareAmp extends Command {
  Climber climber;
  Intake intake;
  Feeder feeder;
  Lights lights;
  Peripherals peripherals;
  TOF tof;
  private double noteTime = -1;
  private boolean commandFinished = false;
  private boolean hitFeeder = false;
  public PrepareAmp(Climber climber, Intake intake, Feeder feeder, Lights lights, Peripherals peripherals, TOF tof) {
    this.climber = climber;
    this.intake = intake;
    this.feeder = feeder;
    this.lights = lights;
    this.peripherals = peripherals;
    this.tof = tof;
  }

  @Override
  public void initialize() {
    climber.prepareAmp = true;
  }

  @Override
  public void execute() {
    if(!climber.intakeRunning) {
      if(tof.getFeederDistMillimeters() <= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM) {
        hitFeeder = true;
        feeder.set(-150);
        climber.setTrapRollerTorque(-20, 0.4);
      } else {
        noteTime = Timer.getFPGATimestamp();
        feeder.set(0);
        if(hitFeeder) {
          climber.setTrapRollerTorque(-20, 0.4);
        } else {
          climber.setTrapRollerTorque(20, 0.4);
        }
      }
    }
    if(noteTime != -1 && Timer.getFPGATimestamp() - noteTime >= 0.5) {
      commandFinished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.prepareAmp = false;
    feeder.set(0);
    climber.setTrapRollerPercent(0);
  }

  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}
