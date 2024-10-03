// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignedPresetShoot extends ParallelRaceGroup {
  /** Creates a new AlignedPresetShoot. */

  public AlignedPresetShoot(Shooter shooter, Feeder feeder, Drive drive, Peripherals peripherals, double[] leftRightAngleTheta) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double[] leftRightAngle = {leftRightAngleTheta[0], leftRightAngleTheta[1], leftRightAngleTheta[2]};
    addCommands(
      new PresetShoot(shooter, feeder, leftRightAngle),
      new DriveThetaAligned(drive, peripherals, shooter, leftRightAngleTheta[3]),
      new EndAfterA()
    );
  }
}
