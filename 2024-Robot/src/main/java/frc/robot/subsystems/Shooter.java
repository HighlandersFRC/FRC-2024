package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ShooterDefault;

public class Shooter extends SubsystemBase {

  private final CANcoder angleEncoder = new CANcoder(Constants.CANInfo.SHOOTER_ANGLE_CANCODER_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final CANcoderConfiguration angleEncoderConfiguration = new CANcoderConfiguration();

  private final TalonFX angleFalcon = new TalonFX(Constants.CANInfo.SHOOTER_ANGLE_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration angleFalconConfiguration = new TalonFXConfiguration();
  // private final DynamicMotionMagicTorqueCurrentFOC
  // angleFalconMotionProfileRequest = new DynamicMotionMagicTorqueCurrentFOC(0,
  // 0.5, 0, 0, 0, 0, false, false, false);
  private final DynamicMotionMagicVoltage angleFalconMotionProfileRequest = new DynamicMotionMagicVoltage(0, 0.5, 0, 0,
      true, 0, 0, false, false, false);
  private final TorqueCurrentFOC angleFalconTorqueRequest = new TorqueCurrentFOC(0, 0, 0, false, false, false);

  private final TalonFX flywheelFalconMaster = new TalonFX(Constants.CANInfo.SHOOTER_FLYWHEEL_MASTER_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration flywheelFalconConfiguration = new TalonFXConfiguration();
  private final TalonFX flywheelFalconFollower = new TalonFX(Constants.CANInfo.SHOOTER_FLYWHEEL_FOLLOWER_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false,
      false, false);
  private final TorqueCurrentFOC flywheelTorqueRequest = new TorqueCurrentFOC(0, 0, 0, false, false, false);

  private final double angleFalconJerk = 15;
  private final double angleFalconAcceleration = 3;
  private final double angleFalconCruiseVelocity = 0.75;

  private final double angleFalconProfileScalarFactor = 1;

  /**
   * Constructs a new Shooter object and sets its default command to ShooterDefault.
   */
  public Shooter() {
    setDefaultCommand(new ShooterDefault(this));
  }

  /**
   * Checks the status of shooter components connected via CAN.
   * 
   * @return true if all components have their sticky faults cleared, false otherwise.
   */
  public boolean getShooterCAN() {
    if (angleEncoder.clearStickyFault_BadMagnet() == StatusCode.OK
        && angleFalcon.clearStickyFault_BootDuringEnable() == StatusCode.OK
        && flywheelFalconMaster.clearStickyFault_BootDuringEnable() == StatusCode.OK
        && flywheelFalconFollower.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      return true;
    } else
      return false;
  }

  public void init() {
    this.angleEncoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    this.angleEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    this.angleFalconConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    this.angleFalconConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.SetPoints.SHOOTER_MAX_ANGLE_ROT
        + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT;
    this.angleFalconConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    this.angleFalconConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.SetPoints.SHOOTER_DOWN_ANGLE_ROT
        + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT;

    this.angleFalconConfiguration.Slot0.kP = 350;
    this.angleFalconConfiguration.Slot0.kI = 0;
    this.angleFalconConfiguration.Slot0.kD = 0;
    this.angleFalconConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    this.angleFalconConfiguration.Slot0.kG = 0;
    this.angleFalconConfiguration.MotionMagic.MotionMagicJerk = this.angleFalconJerk;
    this.angleFalconConfiguration.MotionMagic.MotionMagicAcceleration = this.angleFalconAcceleration;
    this.angleFalconConfiguration.MotionMagic.MotionMagicCruiseVelocity = this.angleFalconCruiseVelocity;
    this.angleFalconConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    this.angleFalconConfiguration.Feedback.FeedbackRemoteSensorID = Constants.CANInfo.SHOOTER_ANGLE_CANCODER_ID;
    this.angleFalconConfiguration.Feedback.SensorToMechanismRatio = 3;
    this.angleFalconConfiguration.Feedback.RotorToSensorRatio = Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO;
    this.angleFalconConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.angleFalconConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.angleFalconConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.angleFalconConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.angleFalcon.getConfigurator().apply(this.angleFalconConfiguration);
    this.angleFalcon.setNeutralMode(NeutralModeValue.Brake);
    this.angleFalcon.setPosition(this.angleEncoder.getAbsolutePosition().getValue());
    this.angleFalcon.set(0);

    this.flywheelFalconConfiguration.Slot0.kP = 12;
    this.flywheelFalconConfiguration.Slot0.kI = 0;
    this.flywheelFalconConfiguration.Slot0.kD = 0;
    this.flywheelFalconConfiguration.Slot0.kS = 1;
    this.flywheelFalconConfiguration.Slot0.kV = 0.2;
    this.flywheelFalconConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.flywheelFalconConfiguration.CurrentLimits.StatorCurrentLimit = 80;
    this.flywheelFalconConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.flywheelFalconConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.flywheelFalconMaster.getConfigurator().apply(this.flywheelFalconConfiguration);
    this.flywheelFalconMaster.setNeutralMode(NeutralModeValue.Coast);
    this.flywheelFalconFollower.getConfigurator().apply(this.flywheelFalconConfiguration);
    this.flywheelFalconFollower.setNeutralMode(NeutralModeValue.Coast);

    // this.flywheelFalconFollower.setControl(new Follower(Constants.CANInfo.SHOOTER_FLYWHEEL_MASTER_MOTOR_ID, true));
  }

  /**
   * Set the state of the shooter, including elevation angle and flywheel velocity.
   *
   * @param degrees The desired elevation angle of the shooter in degrees.
   * @param RPM     The desired flywheel velocity in revolutions per minute (RPM).
   */
  public void set(double degrees, double RPM) {
    double motionProfileScalar = (1.0 - this.angleFalconProfileScalarFactor)
        * Math.cos(Math.toRadians(getAngleDegrees())) + this.angleFalconProfileScalarFactor;
    // System.out.println("Scalar: " + motionProfileScalar);
    if (degrees > Constants.SetPoints.SHOOTER_MAX_ANGLE_DEG) {
      this.angleFalcon.setControl(this.angleFalconMotionProfileRequest
          .withPosition(Constants.SetPoints.SHOOTER_MAX_ANGLE_ROT + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT)
          .withAcceleration(this.angleFalconAcceleration * motionProfileScalar)
          .withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    } else if (degrees < Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG) {
      this.angleFalcon.setControl(this.angleFalconMotionProfileRequest
          .withPosition(Constants.SetPoints.SHOOTER_DOWN_ANGLE_ROT + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT)
          .withAcceleration(this.angleFalconAcceleration * motionProfileScalar)
          .withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    } else {
      this.angleFalcon.setControl(this.angleFalconMotionProfileRequest
          .withPosition(Constants.degreesToRotations(degrees) + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT)
          .withAcceleration(this.angleFalconAcceleration * motionProfileScalar)
          .withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    }
    this.flywheelFalconMaster.setControl(this.flywheelVelocityRequest
        .withVelocity(Constants.RPMToRPS(RPM) * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO));
    this.flywheelFalconFollower.setControl(this.flywheelVelocityRequest
        .withVelocity(Constants.RPMToRPS(-RPM) * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO));
  }

  /**
   * Set the elevation angle of the shooter in rotations.
   *
   * @param degrees The desired elevation angle of the shooter in degrees.
   */
  public void setAngle(double degrees) {
    double motionProfileScalar = (1 - this.angleFalconProfileScalarFactor) * Math.cos(Math.toRadians(getAngleDegrees()))
        + this.angleFalconProfileScalarFactor;
    if (degrees > Constants.SetPoints.SHOOTER_MAX_ANGLE_DEG) {
      this.angleFalcon.setControl(this.angleFalconMotionProfileRequest
          .withPosition(Constants.SetPoints.SHOOTER_MAX_ANGLE_ROT + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT)
          .withAcceleration(this.angleFalconAcceleration * motionProfileScalar)
          .withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    } else if (degrees < Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG) {
      this.angleFalcon.setControl(this.angleFalconMotionProfileRequest
          .withPosition(Constants.SetPoints.SHOOTER_DOWN_ANGLE_ROT + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT)
          .withAcceleration(this.angleFalconAcceleration * motionProfileScalar)
          .withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    } else {
      this.angleFalcon.setControl(this.angleFalconMotionProfileRequest
          .withPosition(Constants.degreesToRotations(degrees) + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT)
          .withAcceleration(this.angleFalconAcceleration * motionProfileScalar)
          .withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    }
  }

  /**
   * Set the shooter flywheel speed
   * @param RPM - speed in rpm of the shooter
   */
  public void setFlywheelRPM(double RPM) {
    this.flywheelFalconMaster.setControl(this.flywheelVelocityRequest
        .withVelocity(Constants.RPMToRPS(RPM) * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO));
    this.flywheelFalconFollower.setControl(this.flywheelVelocityRequest
        .withVelocity(Constants.RPMToRPS(-RPM) * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO));
  }

  /**
   * Set the flywheel percent
   * @param percent - percent output to the motor [-1, 1]
   */
  public void setFlywheelPercent(double percent) {
    this.flywheelFalconMaster.set(percent);
    this.flywheelFalconFollower.set(-percent);
  }

  /**
   * Set the flywheel to torque control
   * @param current - Current to set the motor to (amps)
   * @param maxPercent - Maximum motor power [-1, 1]
   */
  public void setFlywheelTorque(double current, double maxPercent) {
    this.flywheelFalconMaster
        .setControl(this.flywheelTorqueRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }
  /**
   * Set the shooter angle motor to a percent
   * @param percent - Percent output [-1, 1]
   */
  public void setAnglePercent(double percent) {
    this.angleFalcon.set(percent);
  }

  /**
   * Set the shooter angle to torque control
   * @param current - Current to set the motor to (amps)
   * @param maxPercent - Maximum motor power [-1, 1]
   */
  public void setAngleTorque(double current, double maxPercent) {
    this.angleFalcon.setControl(this.angleFalconTorqueRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  /**
   * Get the velocity of the Flywheel
   * @return Flywheel Velocity (RPMs)
   */
  public double getFlywheelRPM() {
    return Constants.RPSToRPM(
        this.flywheelFalconMaster.getVelocity().getValueAsDouble() / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO);
  }

  public double getFlywheelMasterRPM(){
    return Constants.RPSToRPM(
        this.flywheelFalconMaster.getVelocity().getValueAsDouble() / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO);
  }

  /**
   * Get the velocity of the Flywheel Follower
   * @return Flywheel Follower Velocity (RPMs)
   */
  public double getFlywheelFollowerRPM() {
    return -Constants.RPSToRPM(
        this.flywheelFalconFollower.getVelocity().getValueAsDouble() / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO);
  }

  /**
   * Get the shooter angle in rotations
   * @return Shooter angle in rotations
   */
  public double getAngleRotations() {
    return this.angleFalcon.getPosition().getValueAsDouble() - Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT;
  }
  /**
   * Get the shooter angle in degrees
   * @return Shooter angle in degrees
   */
  public double getAngleDegrees() {
    return Constants.rotationsToDegrees(getAngleRotations());
  }

  public double getP() {
    return this.flywheelFalconConfiguration.Slot0.kP;
  }

  public double getI() {
    return this.flywheelFalconConfiguration.Slot0.kI;
  }

  public double getD() {
    return this.flywheelFalconConfiguration.Slot0.kD;
  }

  // Constantly set flywheel velocity PID
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Flywheel RPM", getFlywheelRPM());
    // Logger.recordOutput("Flywheel RPM", getFlywheelRPM());
    SmartDashboard.putNumber("Flywheel %", this.flywheelFalconMaster.getTorqueCurrent().getValueAsDouble());
    // Logger.recordOutput("Flywheel %", this.flywheelFalconMaster.getTorqueCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("Flywheel %Out Master",
    // this.flywheelVortexMaster.getAppliedOutput());
    // SmartDashboard.putNumber("Flywheel %Out Follower",
    // this.flywheelVortexFollower.getAppliedOutput());
    // SmartDashboard.putNumber("Shooter Angle %",
    // this.angleFalcon.getClosedLoopOutput().getValueAsDouble());
  }

  @Override
  public void periodic() {
    boolean shooterEncoder = false;
    boolean shooterAngle = false;

    double newPIDP = SmartDashboard.getNumber("Flywheel P value", getP());
    this.flywheelFalconConfiguration.Slot0.kP = newPIDP;

    double newPIDI = SmartDashboard.getNumber("Flywheel I value", getI());
    this.flywheelFalconConfiguration.Slot0.kI = newPIDI;

    double newPIDD = SmartDashboard.getNumber("Flywheel D value", getD());
    this.flywheelFalconConfiguration.Slot0.kD = newPIDD;

    if (angleEncoder.getSupplyVoltage().getValue() != 0) {
      shooterEncoder = true;
    }
    if (angleFalcon.getSupplyVoltage().getValue() != 0) {
      shooterAngle = true;
    }

    SmartDashboard.putNumber("Shooter Angle Deg", getAngleDegrees());
    SmartDashboard.getNumber("Flywheel P value", getP());
    SmartDashboard.getNumber("Flywheel I value", getI());
    SmartDashboard.getNumber("Flywheel D value", getD());
    SmartDashboard.putBoolean(" Shooter Encoder", shooterEncoder);
    // Logger.recordOutput("Shooter Angle", getAngleDegrees());
    // Logger.recordOutput("Shooter Encoder Online?", shooterEncoder);
    SmartDashboard.putBoolean(" Shooter Angle Motor", shooterAngle);
    // Logger.recordOutput("Shooter Angle Motor Online?", shooterEncoder);
    // Logger.recordOutput("Shooter Angle Setpoint", angleFalcon.getClosedLoopReference().getValueAsDouble());
    // Logger.recordOutput("Shooter Velocity Setpoint", flywheelFalconMaster.getClosedLoopReference().getValueAsDouble());
  }
}
