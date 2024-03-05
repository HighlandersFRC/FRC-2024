package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;

public class ClimberDefault extends Command {
  private Climber climber;
  private TOF tof;

  private boolean isZeroed = false;
  private int numTimesHitBottom = 0;

  private boolean haveNote;
  private boolean haveCarriageNote;

  public ClimberDefault(Climber climber, TOF tof) {
    this.climber = climber;
    this.tof = tof;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    this.isZeroed = false;
    this.numTimesHitBottom = 0;

    this.haveNote = false;
    this.haveCarriageNote = false;
  }

  @Override
  public void execute() {

    if (Math.abs(this.climber.getElevatorVelocityMPS()) < 0.01){
      this.numTimesHitBottom ++;
    }

    if (this.numTimesHitBottom > 2){
      this.isZeroed = true;
      this.numTimesHitBottom = 0;
      this.climber.zeroElevator();
    }

    if (Math.abs(this.climber.getElevatorPositionMeters()) > 0.05){
      this.isZeroed = false;
      this.numTimesHitBottom = 0;
    }

    if (this.tof.getCarriageDistMillimeters() <= Constants.SetPoints.CARRIAGE_TOF_THRESHOLD_MM){
      this.haveCarriageNote = true;
    }

    if (this.tof.getFeederDistMillimeters() <= Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM){
      this.haveNote = true;
    }

    if (this.isZeroed){
      this.climber.setElevatorTorque(0, 0.1);
      if (this.haveCarriageNote && !this.haveNote){
        this.climber.setTrapRollerTorque(20, 0.2);
        this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kFEED);
      } else if (this.haveNote){
        this.climber.setTrapRollerTorque(5, 0.1);
        this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kDOWN);
      } else {
        this.climber.setTrapRollerTorque(20, 0.1);
        this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kDOWN);
      }
    } else {
      this.climber.setTrapRollerPercent(0);
      this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kFEED);
      if (Math.abs(this.climber.getCarriageRotationDegrees() - Constants.SetPoints.CarriageRotation.kFEED.degrees) < 6){
        this.climber.setElevatorTorque(-5, 0.45);
      } else {
        this.climber.setElevatorTorque(0, 0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}