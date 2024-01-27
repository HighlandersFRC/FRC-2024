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
    this.rollerVortexPID.setP(0.000125, 0);
    this.rollerVortexPID.setI(0, 0);
    this.rollerVortexPID.setD(0.02, 0);
    this.rollerVortexPID.setFF(0.000175, 0);
    this.rollerVortexPID.setOutputRange(-1, 1);
    this.rollerVortexEncoder.setPositionConversionFactor(1);
    this.rollerVortexEncoder.setVelocityConversionFactor(1);
    this.rollerVortex.setIdleMode(IdleMode.kBrake);
    this.rollerVortex.set(0);
  }

  //Set roller velocity in RPM
  public void set(double RPM){
    SmartDashboard.putNumber("Feeder Target", RPM);
    this.rollerVortexPID.setReference(RPM * Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO, CANSparkBase.ControlType.kVelocity);
  }

  //Set roller output in percent
  public void setPercent(double percent){
    rollerVortex.set(percent);
  }

  public double getRPM(){
    return this.rollerVortexEncoder.getVelocity() / Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO;
  }

  //Constantly set roller velocity PID
  public void teleopPeriodic(){
    SmartDashboard.putNumber("Feeder RPM", getRPM());
    
  }

  @Override
  public void periodic() {}
}
