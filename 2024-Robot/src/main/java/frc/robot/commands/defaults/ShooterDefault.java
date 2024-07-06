package frc.robot.commands.defaults;

import org.apache.commons.math3.analysis.function.Constant;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Shooter;

public class ShooterDefault extends Command {
  Shooter shooter;

  public ShooterDefault(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (OI.operatorViewButton.getAsBoolean()){
      this.shooter.setFlywheelRPM(1500);
    } else {
      this.shooter.setFlywheelPercent(0);
    }

    if (Math.abs(this.shooter.getAngleDegrees() - Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG) < 2){
      // System.out.println("stopped");
      this.shooter.setAnglePercent(0);
    } else {
      // System.out.println("going down");
      this.shooter.setAngleTorque(-15, 0.45);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
