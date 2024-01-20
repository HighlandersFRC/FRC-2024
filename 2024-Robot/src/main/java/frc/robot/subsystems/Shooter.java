package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ShooterDefault;

public class Shooter extends SubsystemBase {
  private Lights lights;

  private final TalonFX angleFalcon = new TalonFX(Constants.CANInfo.SHOOTER_ANGLE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);

  public Shooter(Lights lights) {
    this.lights = lights;
    setDefaultCommand(new ShooterDefault(this));
  }

  public void init(){

  }

  public void teleopPeriodic(){

  }

  @Override
  public void periodic() {}
}
