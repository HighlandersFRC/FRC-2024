// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Amp extends ParallelRaceGroup {
  /** Creates a new Amp. */
  
    Shooter shooter;
    Drive drive;
    Peripherals peripherals;

  public Amp(Shooter shooter, Drive driver, Peripherals peripherals) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.shooter = shooter;
    this.drive = driver;
    this.peripherals = peripherals;

    addCommands(
      new AngleShooter(shooter, Constants.SetPoints.SHOOTER_AMP_ANGLE_PRESET_DEG),
      new DriveAutoAligned(drive, peripherals),
      new EndOnRT()
    );
  }
}
