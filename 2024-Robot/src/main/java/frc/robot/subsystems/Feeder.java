package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  double startTime;
  // private final CANSparkFlex rollerVortex = new CANSparkFlex(Constants.CANInfo.FEEDER_ROLLER_MOTOR_ID, MotorType.kBrushless);
  // private final RelativeEncoder rollerVortexEncoder = rollerVortex.getEncoder();
  // private final SparkPIDController rollerVortexPID = rollerVortex.getPIDController();
  private final TalonFX rollerFalcon = new TalonFX(Constants.CANInfo.FEEDER_ROLLER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration rollerFalconConfiguration = new TalonFXConfiguration();
  private final VelocityTorqueCurrentFOC rollerFalconVelocityRequest = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  public Feeder() {
    setDefaultCommand(new FeederDefault(this));
  }

  public void init(){
    // this.rollerVortex.restoreFactoryDefaults();
    // this.rollerVortexPID.setP(0.000125, 0);
    // this.rollerVortexPID.setI(0, 0);
    // this.rollerVortexPID.setD(0.02, 0);
    // this.rollerVortexPID.setFF(0.000175, 0);
    // this.rollerVortexPID.setOutputRange(-1, 1);
    // this.rollerVortexEncoder.setPositionConversionFactor(1);
    // this.rollerVortexEncoder.setVelocityConversionFactor(1);
    // this.rollerVortex.setIdleMode(IdleMode.kBrake);
    // this.rollerVortex.set(0);
    this.rollerFalconConfiguration.Slot0.kP = 12.0;
    this.rollerFalconConfiguration.Slot0.kI = 0.0;
    this.rollerFalconConfiguration.Slot0.kD = 0.0;
    this.rollerFalconConfiguration.Slot0.kS = 5.0;
    this.rollerFalcon.getConfigurator().apply(this.rollerFalconConfiguration);
    this.rollerFalcon.setNeutralMode(NeutralModeValue.Brake);

  }

  //Set roller velocity in RPM
  public void set(double RPM){
    SmartDashboard.putNumber("Feeder Target", RPM);
    // this.rollerVortexPID.setReference(RPM * Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO, CANSparkBase.ControlType.kVelocity);
    rollerFalcon.setControl(rollerFalconVelocityRequest.withVelocity(Constants.RPMToRPS(RPM) * Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO));
  }

  //Set roller output in percent
  public void setPercent(double percent){
    // rollerVortex.set(percent);
    rollerFalcon.set(percent);
  }

  public double getRPM(){
    // return this.rollerVortexEncoder.getVelocity() / Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO;
    return this.rollerFalcon.getRotorVelocity().getValue() / Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO;
  }

  //Constantly set roller velocity PID
  public void teleopPeriodic(){
   
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder RPM", Constants.RPSToRPM(getRPM()));
  }
}
