package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Intake;

public class IntakeDefault extends Command {
  Intake intake;
  boolean isZeroed = false;
  int numTimesOverCurrentLimit = 0;

  public IntakeDefault(Intake intake) {
    this.intake = intake;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.intake.setRollerPercent(0);
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
    if (Math.abs(this.intake.getAngleCurrent()) > 50 && !this.isZeroed){
      this.intake.setAnglePercent(0);
      this.intake.setAngleEncoderPosition(0);
      this.numTimesOverCurrentLimit ++;
    } else if (!this.isZeroed) {
      // this.intake.setAnglePercent(0.2);
      this.intake.setAnglePercent(0);
    } else {
      this.intake.setAngle(Constants.SetPoints.IntakePosition.kUP);
    }

    if (numTimesOverCurrentLimit > 2){
      this.isZeroed = true;
      this.numTimesOverCurrentLimit = 0;
    }

    if (Math.abs(this.intake.getAngleRotations()) > 0.05){
      this.isZeroed = false;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
