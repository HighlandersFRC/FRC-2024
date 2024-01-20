package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeDefault extends Command {
  Intake intake;

  public IntakeDefault(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setIntake(Constants.Setpoints.INTAKE_UP_ANGLE, 0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
