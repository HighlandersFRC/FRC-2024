package frc.robot.commands.defaults;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;

public class ClimberDefault extends Command {
  private Climber climber;
  private TOF tof;
  private Proximity proximity;

  private boolean isZeroed = false;
  private int numTimesHitBottom = 0;

  private boolean haveNote;
  private boolean haveCarriageNote;

  public ClimberDefault(Climber climber, TOF tof, Proximity proximity) {
    this.climber = climber;
    this.tof = tof;
    this.proximity = proximity;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    this.isZeroed = false;
    this.numTimesHitBottom = 0;

    if (this.proximity.getShooterProximity()){
      this.haveNote = true;
    } else {
      this.haveNote = false;
    }
    this.haveCarriageNote = false;
  }

  @Override
  public void execute() {
    this.climber.setElevatorPercent(0.0);
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

    if (this.proximity.getCarriageProximity()){
      this.haveCarriageNote = true;
    }

    if (this.proximity.getShooterProximity()){
      this.haveNote = true;
    }

    System.out.println("have note climber: " + haveNote);
    // SmartDashboard.putBoolean("have note climber", haveNote);
    System.out.println("carriage have note climber: " + haveCarriageNote);
    if (this.isZeroed){
      // this.climber.setElevatorTorque(0, 0.1);
      if (this.haveCarriageNote && !this.haveNote){
        this.climber.setTrapRollerTorque(20, 0.2);
        this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kFEED);
        System.out.println("in carriage");
      } else if (this.haveNote){
        this.climber.setTrapRollerTorque(5, 0.1);
        this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kDOWN.degrees - 3);
        System.out.println("in feeder proximity");
      } else {
        this.climber.setTrapRollerTorque(20, 0.1);
        this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kDOWN.degrees - 3);
        System.out.println("out of proximity");
      }
    } else {
      this.climber.setTrapRollerPercent(0);
      this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kFEED);
      // if (Math.abs(this.climber.getCarriageRotationDegrees() - Constants.SetPoints.CarriageRotation.kFEED.degrees) < 6){
      //   this.climber.setElevatorTorque(-5, 0.45);
      // } else {
      //   this.climber.setElevatorTorque(0, 0);
      // }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}