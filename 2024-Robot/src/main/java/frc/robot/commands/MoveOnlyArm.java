package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveOnlyArm extends Command {
  private Climber climber;
  private double armDegrees;
  public MoveOnlyArm(Climber climber, double armDegrees) {
    this.climber = climber;
    this.armDegrees = armDegrees;
    addRequirements(this.climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.climber.setElevatorPercent(0);
    this.climber.setCarriageRotationDegrees(this.armDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
