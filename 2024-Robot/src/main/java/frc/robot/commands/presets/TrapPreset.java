package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.IndexNoteToCarriage;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunClimberAndCarriageWithControl;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeTorque;
import frc.robot.commands.RunTrap;
import frc.robot.commands.SetCarriage;
import frc.robot.commands.SetCarriageWithControl;
import frc.robot.commands.SetClimber;
import frc.robot.commands.SetClimberWithoutIntake;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TrapPreset extends SequentialCommandGroup {
  public TrapPreset(Climber climber, Feeder feeder, Intake intake, Proximity proximity, Shooter shooter) {
    addRequirements(climber, feeder, intake, shooter);
    addCommands(
      new IndexNoteToCarriage(feeder, climber, intake, proximity, shooter, 0.6),
      new ParallelDeadlineGroup(
        new SetClimberWithoutIntake(climber, Constants.SetPoints.ElevatorPosition.kAMP, Constants.SetPoints.CarriageRotation.kFEED),
        new RunFeeder(feeder, -150),
        new RunIntakeTorque(intake, 15, 0.6, 60)
      ),
      new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kTRAP, 0, 0, true),
      new RunClimberAndCarriageWithControl(climber, Constants.SetPoints.CarriageRotation.kTRAP, 0, 0, 40, 0.7)
    );
  }
}
