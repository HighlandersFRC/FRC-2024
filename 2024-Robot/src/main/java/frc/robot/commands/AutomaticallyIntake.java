// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutomaticallyIntake extends ParallelDeadlineGroup {
  Drive drive;
  Peripherals peripherals;
  Intake intake;
  Feeder feeder;
  Shooter shooter;
  /** Creates a new AutomaticallyIntake. */
  public AutomaticallyIntake(Drive drive, Peripherals peripherals, Intake intake, Feeder feeder, Shooter shooter) {
    
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new RunIntake(intake, feeder, shooter, 0.4));
    // this.drive = drive;
    // this.peripherals = peripherals;
    // this.intake = intake;
    // this.feeder = feeder;
    addCommands(new MoveToPiece(drive, peripherals, intake));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
