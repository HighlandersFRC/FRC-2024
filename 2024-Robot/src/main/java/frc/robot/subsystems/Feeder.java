package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.FeederDefault;

public class Feeder extends SubsystemBase {
  private Lights lights;
  double startTime;
  private final CANSparkFlex rollerVortex = new CANSparkFlex(Constants.CANInfo.FEEDER_ROLLER_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder rollerVortexEncoder = rollerVortex.getEncoder();
  private final SparkPIDController rollerVortexPID = rollerVortex.getPIDController();

  public Feeder(Lights lights) {
    this.lights = lights;
    setDefaultCommand(new FeederDefault(this));
  }

  public void init(){
    this.rollerVortex.restoreFactoryDefaults();
    this.rollerVortexPID.setP(0.00012, 0);
    this.rollerVortexPID.setI(0.00000, 0);
    this.rollerVortexPID.setD(0, 0);
    this.rollerVortexPID.setFF(0.00011, 0);
    this.rollerVortexPID.setOutputRange(-1, 1);
    this.rollerVortexEncoder.setPositionConversionFactor(1);
    this.rollerVortexEncoder.setVelocityConversionFactor(1);
    this.rollerVortex.setIdleMode(IdleMode.kBrake);
    this.rollerVortex.set(0);
  }

  //Set roller velocity in RPM
  public void setFeeder(double rollerVelocity){
    this.rollerVortexPID.setReference(rollerVelocity * Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO, CANSparkBase.ControlType.kVelocity);
  }

  //Set roller output in percent
  public void setFeederPercent(double percent){
    rollerVortex.set(percent);
  }

  public double getFeederVelocity(){
    return this.rollerVortexEncoder.getVelocity() / Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO;
  }

  //Constantly set roller velocity PID
  public void teleopPeriodic(){
    SmartDashboard.putNumber("Feeder", getFeederVelocity());
  }

  @Override
  public void periodic() {}
}
