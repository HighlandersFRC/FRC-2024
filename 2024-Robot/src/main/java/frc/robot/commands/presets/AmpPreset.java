package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IndexNoteToCarriage;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIntakeTorque;
import frc.robot.commands.SetCarriageWithControl;
import frc.robot.commands.SetClimberWithoutIntake;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AmpPreset extends SequentialCommandGroup {
  public AmpPreset(Climber climber, Feeder feeder, Intake intake, Proximity proximity, Shooter shooter) {
    addRequirements(climber, feeder, intake, shooter);
    addCommands(
      new IndexNoteToCarriage(feeder, climber, intake, proximity, shooter, 0.65),
      new ParallelDeadlineGroup(
        new SetClimberWithoutIntake(climber, Constants.SetPoints.ElevatorPosition.kAMP, Constants.SetPoints.CarriageRotation.kFEED),
        new RunFeeder(feeder, -150),
        new RunIntakeTorque(intake, 15, 0.6, 60)
      ),
      new SetCarriageWithControl(climber, Constants.SetPoints.CarriageRotation.kAMP, 0, 0, false)
    );
  }
}