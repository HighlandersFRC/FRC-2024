package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import java.lang.annotation.Documented;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
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
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;

public class Feeder extends SubsystemBase {
  double startTime;
  // private final CANSparkFlex rollerVortex = new CANSparkFlex(Constants.CANInfo.FEEDER_ROLLER_MOTOR_ID, MotorType.kBrushless);
  // private final RelativeEncoder rollerVortexEncoder = rollerVortex.getEncoder();
  // private final SparkPIDController rollerVortexPID = rollerVortex.getPIDController();
  private final TalonFX rollerFalcon = new TalonFX(Constants.CANInfo.FEEDER_ROLLER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration rollerFalconConfiguration = new TalonFXConfiguration();
  private final VelocityTorqueCurrentFOC rollerFalconVelocityRequest = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  private final TorqueCurrentFOC rollerFalconTorqueRequest = new TorqueCurrentFOC(0, 0, 0, false, false, false);

  /**
   * Constructs a new instance of the Feeder class.
   *
   * @param tof The Time-of-Flight (TOF) sensor used by the Feeder.
  */
  public Feeder(TOF tof, Proximity proximity) {
    setDefaultCommand(new FeederDefault(this,tof,proximity));
  }

  /**
   * Checks if the feeder CAN is available.
   *
   * @return {@code true} if the feeder CAN is available, {@code false} otherwise.
  */
  public boolean getFeederCAN() {
    if(rollerFalcon.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      return true;
    } else return false;
  }

  public void init(){
    this.rollerFalconConfiguration.Slot0.kP = 12.5;
    this.rollerFalconConfiguration.Slot0.kI = 0.0;
    this.rollerFalconConfiguration.Slot0.kD = 0.0;
    this.rollerFalconConfiguration.Slot0.kS = 0.0;
    this.rollerFalconConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.rollerFalconConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.rollerFalconConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.rollerFalconConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.rollerFalcon.getConfigurator().apply(this.rollerFalconConfiguration);
    this.rollerFalcon.setNeutralMode(NeutralModeValue.Brake);

  }

  /**
   * Sets the velocity of the feeder roller.
   *
   * @param RPM The desired velocity of the feeder roller in RPM (Revolutions Per Minute).
  */
  public void set(double RPM){
    rollerFalcon.setControl(rollerFalconVelocityRequest.withVelocity(Constants.RPMToRPS(RPM) * Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO));
  }

  /**
   * Sets the output percentage of the roller Falcon.
   *
   * @param percent The desired output percentage of the roller Falcon. Should be a value between 0 and 1.
  */
  public void setPercent(double percent){
    rollerFalcon.set(percent);
  }

  /**
   * Sets the torque control parameters for the roller Falcon.
   *
   * @param current The desired current output in amps.
   * @param maxPercent The maximum absolute duty cycle percentage.
  */
  public void setTorque(double current, double maxPercent){
    this.rollerFalcon.setControl(this.rollerFalconTorqueRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  /**
   * Retrieves the rotational speed of the feeder roller in Rotations Per Second (RPS).
   *
   * @return The rotational speed of the feeder roller in RPS.
  */
  public double getRPS(){
    return this.rollerFalcon.getRotorVelocity().getValue() / Constants.Ratios.FEEDER_ROLLER_GEAR_RATIO;
  }
  
  public void teleopPeriodic(){}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder RPM", Constants.RPSToRPM(getRPS()));
    // Logger.recordOutput("Feeder RPM", Constants.RPSToRPM(getRPS()));
  }
}
