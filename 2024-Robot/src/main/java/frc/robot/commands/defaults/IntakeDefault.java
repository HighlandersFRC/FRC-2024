package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Intake;

public class IntakeDefault extends Command {
  private Intake intake;
  private boolean isZeroed = false;
  private int numTimesOverCurrentLimit = 0;
  private double initTime;

  public IntakeDefault(Intake intake) {
    this.intake = intake;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
    this.isZeroed = false;
    this.numTimesOverCurrentLimit = 0;
    this.initTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    if (OI.getOperatorLB()){
      this.intake.setRollers(0);
    } else {
      this.intake.setRollers(250);
    }
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);

    if (Timer.getFPGATimestamp() - this.initTime < 0.6){
      this.intake.setAngle(Constants.SetPoints.IntakePosition.kUP.degrees);
    } else {
      if (Math.abs(this.intake.getAngleRPS()) < 0.01 && !this.isZeroed){
        this.intake.setAngleTorqueCurrent(10, 0.1);
        this.intake.setAngleEncoderPosition(0);
        this.numTimesOverCurrentLimit ++;
      } else if (!this.isZeroed) {
        this.intake.setAngleTorqueCurrent(45, 0.3);
      } else {
        this.intake.setAngleTorqueCurrent(5, 0.1);
      }

      if (numTimesOverCurrentLimit > 3){
        this.isZeroed = true;
        this.numTimesOverCurrentLimit = 0;
      }

      if (Math.abs(this.intake.getAngleRotations()) > 0.05){
        this.isZeroed = false;
      }
    }
    // this.intake.setAnglePercent(0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
