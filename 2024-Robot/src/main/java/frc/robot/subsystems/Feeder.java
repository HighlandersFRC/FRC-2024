package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.FeederDefault;

public class Feeder extends SubsystemBase {
  private Lights lights;

  private final CANSparkFlex rollerVortex = new CANSparkFlex(Constants.CANInfo.FEEDER_ROLLER_MOTOR_ID, MotorType.kBrushless);
  private final SparkAbsoluteEncoder rollerVortexEncoder = rollerVortex.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController rollerVortexPID = rollerVortex.getPIDController();

  private double rollerVortexVelocitySetpoint = 0;

  public Feeder(Lights lights) {
    this.lights = lights;
    setDefaultCommand(new FeederDefault(this));
  }

  public void init(){
    this.rollerVortexVelocitySetpoint = 0;

    this.rollerVortex.restoreFactoryDefaults();
    this.rollerVortexPID.setP(0.00012, 0);
    this.rollerVortexPID.setI(0.000001, 0);
    this.rollerVortexPID.setD(0, 0);
    this.rollerVortexPID.setFF(0.00011, 0);
    this.rollerVortexPID.setOutputRange(-1, 1);
    this.rollerVortexEncoder.setPositionConversionFactor(1);
    this.rollerVortexEncoder.setVelocityConversionFactor(1);
    this.rollerVortex.setIdleMode(IdleMode.kBrake);
  }

  //Set roller velocity in RPM
  public void setFeeder(double rollerVelocity){
    this.rollerVortexVelocitySetpoint = rollerVelocity;
  }

  //Constantly set roller velocity PID
  public void teleopPeriodic(){
    this.rollerVortexPID.setReference(this.rollerVortexVelocitySetpoint * Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO, CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void periodic() {}
}
