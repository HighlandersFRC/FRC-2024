package frc.robot.commands.defaults;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
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
  private boolean haveFeederNote;

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
    this.haveFeederNote = false;
  }

  @Override
  public void execute() {
    this.climber.setElevatorPercent(0.0);
    if (Math.abs(this.climber.getElevatorVelocityMPS()) < 0.01){
      this.numTimesHitBottom ++;
    }

    if (!this.climber.getElevatorLimitSwitch()){
      this.isZeroed = true;
      this.numTimesHitBottom = 0;
      this.climber.zeroElevator();
      // System.out.println("zero elevator");
    } else {
      this.isZeroed = false;
    }

    if (Math.abs(this.climber.getElevatorPositionMeters()) > 0.05 && this.isZeroed){
      this.isZeroed = false;
      this.numTimesHitBottom = 0;
    }

    if (this.proximity.getCarriageProximity()){
      this.haveCarriageNote = true;
    }

    if (this.proximity.getShooterProximity()){
      this.haveNote = true;
    }

    if (this.proximity.getFeederProximity()){
      this.haveFeederNote = true;
    }

    // System.out.println("carriage proximity: " + this.proximity.getCarriageProximity());
    // System.out.println("have note " + haveNote);
    // System.out.println("zeroed: " + isZeroed);
    if (this.isZeroed){
      this.climber.setElevatorTorque(0, 0.1);
      if (OI.operatorController.getLeftBumper()){
        this.climber.setTrapRollerPercent(0);
        this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kFEED);
      } else if (!this.proximity.getCarriageProximity() && this.proximity.getFeederProximity() && !this.proximity.getShooterProximity()){
        // System.out.println("1");
        this.climber.setTrapRollerPercent(0);
        this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kDOWN.degrees);
      } else if (this.proximity.getCarriageProximity() && this.proximity.getFeederProximity() && this.proximity.getShooterProximity()){
        // System.out.println("2");
        this.climber.setTrapRollerTorque(15, 0.1);
        this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kDOWN.degrees);
      } else if (this.haveNote && !this.proximity.getShooterProximity()){
        // System.out.println("3");
        this.climber.setTrapRollerTorque(15, 0.1);
        this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kDOWN.degrees);
      } else if (this.haveCarriageNote && !this.haveNote && this.haveFeederNote){
        // System.out.println("4");
        this.climber.setTrapRollerTorque(15, 0.1);
        this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kDOWN.degrees);
      } else if (this.proximity.getCarriageProximity() && !this.haveNote){
        // System.out.println("5");
        this.climber.setTrapRollerTorque(30, 0.4);
        this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kFEED.degrees - 5);
      } else if (this.haveNote){
        // System.out.println("6");
        this.climber.setTrapRollerTorque(20, 0.2);
        this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kFEED.degrees - 5);
      } else {
        // System.out.println("7");
        this.climber.setTrapRollerTorque(20, 0.1);
        this.climber.setCarriageRotationDegrees(Constants.SetPoints.CarriageRotation.kDOWN.degrees);
      }
    } else {
      this.climber.setTrapRollerPercent(0);
      this.climber.setCarriageRotation(Constants.SetPoints.CarriageRotation.kFEED);

      if (Math.abs(this.climber.getCarriageRotationDegrees() - Constants.SetPoints.CarriageRotation.kFEED.degrees) < 3){
        // System.out.println("8");
        if (this.climber.getElevatorPositionMeters() > 0.1){
          this.climber.setElevatorTorque(-30, 0.7);
        } else {
          this.climber.setElevatorTorque(-10, 0.6);
        }
      } else {
        // System.out.println("9");
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