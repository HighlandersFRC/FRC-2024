package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.controlloops.PID;

public class TurnToTarget extends Command {
  private Drive drive;
  private Peripherals peripherals;

  private PID turnPID;
  private double kP = 0.03;
  private double kI = 0.0;
  private double kD = 0.07;

  private double speakerAngleDegrees;
  private double targetPigeonAngleDegrees;

  private double initTime;
  private double timeout = 1.5;

  private double robotAngleAllowedErrorDegrees = 2;
  private boolean canSeeTag = false;

  public TurnToTarget(Drive drive, Peripherals peripherals) {
    this.drive = drive;
    this.peripherals = peripherals;
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {
    this.initTime = Timer.getFPGATimestamp();
    this.turnPID = new PID(this.kP, this.kI, this.kD);
    this.turnPID.setMinOutput(-3);
    this.turnPID.setMaxOutput(3);
    this.speakerAngleDegrees = 0;
    this.targetPigeonAngleDegrees = this.peripherals.getPigeonAngle();
    this.canSeeTag = false;
  }

  @Override
  public void execute() {
    double pigeonAngleDegrees = this.peripherals.getPigeonAngle();

    ArrayList<Integer> ids = this.peripherals.getFrontCamIDs();

    this.canSeeTag = false;
    for (int id : ids){
      if (id == 7 || id == 4){
        this.canSeeTag = true;
      }
    }

    if (this.canSeeTag){
      this.speakerAngleDegrees = this.peripherals.getFrontCamTargetTx();
      this.targetPigeonAngleDegrees = pigeonAngleDegrees - this.speakerAngleDegrees;
    }

    this.turnPID.setSetPoint(targetPigeonAngleDegrees);
    this.turnPID.updatePID(pigeonAngleDegrees);
    double turnResult = -this.turnPID.getResult();

    this.drive.autoRobotCentricTurn(turnResult);

    System.out.println("Speaker Ang Deg: " + this.speakerAngleDegrees);
    System.out.println("Pigeon Angle: " + pigeonAngleDegrees);
    System.out.println("Turn Result: " + turnResult);
    System.out.println("Can See Tag: " + this.canSeeTag);
  }

  @Override
  public void end(boolean interrupted) {
    this.drive.autoRobotCentricTurn(0);
  }

  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - this.initTime > this.timeout){
      return true;
    } else if (Math.abs(this.speakerAngleDegrees) <= this.robotAngleAllowedErrorDegrees && this.canSeeTag){
      return true;
    } else {
      return false;
    }
  }
}
