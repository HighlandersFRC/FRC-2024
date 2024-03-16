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
  }

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

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
      lights.setCommandRunning(false);
      lights.clearAnimations();
      OI.driverController.setRumble(RumbleType.kBothRumble, 0);
      OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
