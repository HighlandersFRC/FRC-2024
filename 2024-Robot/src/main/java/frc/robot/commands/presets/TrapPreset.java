// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.IndexNoteToCarriage;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunTrap;
import frc.robot.commands.SetCarriage;
import frc.robot.commands.SetClimber;
import frc.robot.commands.SetClimberWithoutIntake;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapPreset extends SequentialCommandGroup {
  /** Creates a new AmpPreset. */
  public TrapPreset(Climber climber, Feeder feeder, Intake intake, TOF tof) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(climber, feeder, intake);
    addCommands(
      new IndexNoteToCarriage(feeder, climber, intake, tof),
      new ParallelDeadlineGroup(
        new SetClimberWithoutIntake(climber, intake, Constants.SetPoints.ElevatorPosition.kUP, Constants.SetPoints.IntakePosition.kDOWN),
        new RunFeeder(feeder, -60),
        new RunIntake(intake, 60)
      )
      // new SetCarriage(climber, Constants.SetPoints.CarriageRotation.kUP, 0, 0)
    );
  }
}
