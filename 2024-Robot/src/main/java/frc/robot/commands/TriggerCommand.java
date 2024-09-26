// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TriggerCommand extends SequentialCommandGroup {
  /** Creates a new TriggerCommand. */
  public TriggerCommand(BooleanSupplier startCondition, Command command, BooleanSupplier endCondition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitForCondition(startCondition),
      new ParallelRaceGroup(
        new NoRequirements(command),
        new WaitForCondition(endCondition)
      )
    );
  }
}
