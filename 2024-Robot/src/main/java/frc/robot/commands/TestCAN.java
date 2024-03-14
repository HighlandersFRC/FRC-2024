// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class TestCAN extends Command {
  /** Creates a new TestLights. */
  private Lights lights;
  private Drive drive;
  private Intake intake;
  private Shooter shooter;
  private Feeder feeder;
  private Climber climber;
  private Peripherals peripherals;
  public TestCAN(Lights lights, Drive drive, Intake intake, Shooter shooter, Feeder feeder, Climber climber, Peripherals peripherals) {
    this.lights = lights;
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.feeder = feeder;
    this.climber = climber;
    this.peripherals = peripherals;
    addRequirements(this.lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lights.setCommandRunning(true);
    lights.clearAnimations();
    if(drive.getSwerveCAN() && shooter.getShooterCAN() && intake.getIntakeCAN() && feeder.getFeederCAN() && climber.getClimberCAN() && peripherals.limelightsConnected()) {
      lights.blinkGreen(-1);
    } else {
      lights.blinkYellow(-1);
      OI.driverController.setRumble(RumbleType.kBothRumble, 0.6);
      OI.operatorController.setRumble(RumbleType.kBothRumble, 0.6);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      lights.setCommandRunning(false);
      lights.clearAnimations();
      OI.driverController.setRumble(RumbleType.kBothRumble, 0);
      OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
