package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.IntakeDefault;
import frc.robot.sensors.TOF;

public class Intake extends SubsystemBase {

  private final TalonFX angleFalcon = new TalonFX(Constants.CANInfo.INTAKE_ANGLE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration angleFalconConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC anglefalconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  private final TorqueCurrentFOC angleFalconCurrentRequest = new TorqueCurrentFOC(0, 0, 0, false, false, false);

  private final TalonFX rollerFalcon = new TalonFX(Constants.CANInfo.INTAKE_ROLLER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration rollerFalconConfiguration = new TalonFXConfiguration();
  private final VelocityTorqueCurrentFOC rollerFalconVelocityRequest = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  /**
   * Constructs a new instance of the Intake class.
  */
  public Intake() {
    setDefaultCommand(new IntakeDefault(this));
  }
  
  /**
   * Checks if intake CAN is available.
   *
   * @return {@code true} if the intake CAN is available, {@code false} otherwise.
  */
  public boolean getIntakeCAN() {
    if(angleFalcon.clearStickyFault_BootDuringEnable() == StatusCode.OK && rollerFalcon.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      return true;
    } else return false;
  }

  public void init(){
    this.angleFalconConfiguration.Slot0.kP = 40;
    this.angleFalconConfiguration.Slot0.kI = 0;
    this.angleFalconConfiguration.Slot0.kD = 6;
    this.angleFalconConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    this.angleFalconConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.angleFalconConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.angleFalconConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.angleFalconConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.angleFalconConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
    this.angleFalconConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
    this.angleFalcon.getConfigurator().apply(this.angleFalconConfiguration);
    this.angleFalcon.setNeutralMode(NeutralModeValue.Brake);
    this.angleFalcon.setPosition(0);
    this.rollerFalconConfiguration.Slot0.kP = 10;    
    this.rollerFalconConfiguration.Slot0.kI = 0;
    this.rollerFalconConfiguration.Slot0.kD = 0.1;
    this.rollerFalconConfiguration.Slot0.kS = 4;
    this.rollerFalconConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.rollerFalconConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.rollerFalconConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.rollerFalconConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.rollerFalconConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
    this.rollerFalconConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
    this.rollerFalcon.getConfigurator().apply(this.rollerFalconConfiguration);
    this.rollerFalcon.setNeutralMode(NeutralModeValue.Coast);
  }

  public void autoInit(){
    this.angleFalcon.setPosition(-0.08);
  }

  /**
   * Sets the intake angle and roller velocity.
   *
   * @param degrees The desired angle of the intake in degrees.
   * @param RPM The desired velocity of the roller in RPM (Revolutions Per Minute).
  */
  public void set(double degrees, double RPM){
    if (degrees < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG){
      // System.out.println("1");
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (degrees > Constants.SetPoints.INTAKE_UP_ANGLE_DEG){
      // System.out.println("2");
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      // System.out.println("3");
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.degreesToRotations(degrees) * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
    // System.out.println("Degrees: " + degrees);
    this.rollerFalcon.setControl(this.rollerFalconVelocityRequest.withVelocity(Constants.RPMToRPS(-RPM) * Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO));
  }

  /**
   * Sets the intake state and position along with the roller velocity.
   *
   * @param position The desired position for the intake (either kUP or kDOWN).
   * @param RPM The desired velocity of the roller in RPM (Revolutions Per Minute).
  */
  public void set(Constants.SetPoints.IntakePosition position, double RPM){
    if (position.degrees < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (position.degrees > Constants.SetPoints.INTAKE_UP_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(position.rotations * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
    this.rollerFalcon.setControl(this.rollerFalconVelocityRequest.withVelocity(Constants.RPMToRPS(-RPM) * Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO));
  }

  /**
   * Sets the intake angle with the specified setpoint.
   *
   * @param position The desired setpoint for the intake angle in degrees.
  */
  public void setAngle(double degrees){
    if (degrees < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (degrees > Constants.SetPoints.INTAKE_UP_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.degreesToRotations(degrees) * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
  }

  /**
   * Sets the intake angle with the specified setpoint.
   *
   * @param position The desired setpoint for the intake angle.
  */
  public void setAngle(Constants.SetPoints.IntakePosition position){
    if (position.degrees < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (position.degrees > Constants.SetPoints.INTAKE_UP_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(position.rotations * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
  }

  /**
   * Sets the velocity of the rollers.
   *
   * @param RPM The desired velocity of the rollers in RPM (Revolutions Per Minute).
  */
  public void setRollers(double RPM){
    this.rollerFalcon.setControl(this.rollerFalconVelocityRequest.withVelocity(Constants.RPMToRPS(-RPM) * Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO));
  }

  /**
   * Set the Angle to a percent
   * 
   * @param percent - Current drawn from motor (amps)
  */
  public void setRollerPercent(double percent){
    this.rollerFalcon.set(-percent);
  }

  /**
   * Set the Angle to a percent
   * 
   * @param percent - Current drawn from motor (amps)
  */
  public void setAnglePercent(double percent){
    this.angleFalcon.set(percent);
  }

  /**
   * Set the Angle of the intake to Torque Current Control
   * 
   * @param current - Current drawn from motor (amps)
   * @param maxPercent - Maximum motor power (from 0 to 1)
  */
  public void setAngleTorqueCurrent(double current, double maxPercent){
    this.angleFalcon.setControl(this.angleFalconCurrentRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  /**
   * Set the current angle of the encoder to a position
   * 
   * @param rotations - The position to set in rotations
  */
  public void setAngleEncoderPosition(double rotations){
    this.angleFalcon.setPosition(rotations * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO);
  }

  /**
   * Get the velocity of intake roller in RPM
   * 
   * @return The velocity of the roller in RPM
  */
  public double getRPM(){
    return this.rollerFalcon.getVelocity().getValue() / Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO;
  }

  /**
   * Get the angle of the intake in rotations
   * 
   * @return The position in rotations of the intake
  */
  public double getAngleRotations(){
    return this.angleFalcon.getPosition().getValue() / Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO;
  }

  /**
   * Get the angle of the intake in degrees
   * 
   * @return The position in degrees of the intake
  */
  public double getAngleDegrees(){
    return Constants.rotationsToDegrees(this.angleFalcon.getPosition().getValue() / Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO);
  }

  /**
   * Get the stator current of the angle motor (amps)
   * 
   * @return The stator current of the angle motor in amps
  */
  public double getAngleCurrent(){
    return this.angleFalcon.getStatorCurrent().getValue();
  }
  
  /**
   * Get the velocity of the intake's angle in RPS 
   * 
   * @return The angular velocity of the intake in RPS
  */
  public double getAngleRPS(){
    return this.angleFalcon.getVelocity().getValueAsDouble();
  }

  public double getRollerCurrent(){
    return this.rollerFalcon.getStatorCurrent().getValueAsDouble();
  }

  // //Get value of intake rotation limit switch
  // public boolean getAngleLimitSwitch(){
  //   if (angleFalcon.getReverseLimit().getValue().value == 1){
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  //Constantly set roller velocity PID
  public void teleopPeriodic(){
    
  }

  @Override
  public void periodic() {
    // boolean angleMotor = false;
    // boolean intakeMotor = false;
    // boolean intakeTOF = false;
    // if(angleFalcon.getMotorVoltage().getValue() != 0.0 ){
    //   angleMotor = true;
    // }
    // if(rollerFalcon.getSupplyVoltage().getValue() != 0.0){
    //   intakeMotor = true;
    // }
    
    
    // SmartDashboard.putBoolean("Intake Angle Motor", angleMotor);
    // SmartDashboard.putBoolean("Intake Roller Motor", intakeMotor);
    // Logger.recordOutput("Intake Angle Motor Online?", angleMotor);
    // Logger.recordOutput("Intake Roller Motor Online?", intakeMotor);
    // Logger.recordOutput("Intake Angle", getAngleDegrees());
    // Logger.recordOutput("Intake Angle Setpoint", Constants.degreesToRotations(angleFalcon.getClosedLoopReference().getValueAsDouble()/Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    // Logger.recordOutput("Intake Roller Velocity", rollerFalcon.getRotorVelocity().getValueAsDouble()/Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO);
    Logger.recordOutput("intakeRollerStator", this.rollerFalcon.getStatorCurrent().getValueAsDouble());
  }
}
