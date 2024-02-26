package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
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

  private final CANcoder angleEncoder = new CANcoder(Constants.CANInfo.SHOOTER_ANGLE_CANCODER_ID, Constants.CANInfo.CANBUS_NAME);
  private final CANcoderConfiguration angleEncoderConfiguration = new CANcoderConfiguration();

  private final TalonFX angleFalcon = new TalonFX(Constants.CANInfo.SHOOTER_ANGLE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration angleFalconConfiguration = new TalonFXConfiguration();
  private final DynamicMotionMagicTorqueCurrentFOC angleFalconPositionMotionProfileRequest = new DynamicMotionMagicTorqueCurrentFOC(0, 0.3, 0, 0, 0, 0, false, false, false);

  private final TalonFX flywheelFalconMaster = new TalonFX(Constants.CANInfo.SHOOTER_FLYWHEEL_MASTER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration flywheelFalconConfiguration = new TalonFXConfiguration();
  private final TalonFX flywheelFalconFollower = new TalonFX(Constants.CANInfo.SHOOTER_FLYWHEEL_FOLLOWER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  private final TorqueCurrentFOC flywheelTorqueRequest = new TorqueCurrentFOC(0, 0, 0, false, false, false);

  private final double angleFalconJerk = 10;
  private final double angleFalconAcceleration = 1.25;
  private final double angleFalconCruiseVelocity = 0.5;

  private final double angleFalconProfileScalarFactor = 0.5;

  public Shooter() {
    setDefaultCommand(new ShooterDefault(this));
  }

  public void init(){
    this.angleEncoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    this.angleEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    this.angleFalconConfiguration.Slot0.kP = 5000;
    this.angleFalconConfiguration.Slot0.kI = 0;
    this.angleFalconConfiguration.Slot0.kD = 60;
    this.angleFalconConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    this.angleFalconConfiguration.Slot0.kG = 7;
    this.angleFalconConfiguration.MotionMagic.MotionMagicJerk = this.angleFalconJerk;
    this.angleFalconConfiguration.MotionMagic.MotionMagicAcceleration = this.angleFalconAcceleration;
    this.angleFalconConfiguration.MotionMagic.MotionMagicCruiseVelocity = this.angleFalconCruiseVelocity;

    this.angleFalconConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    this.angleFalconConfiguration.Feedback.FeedbackRemoteSensorID = Constants.CANInfo.SHOOTER_ANGLE_CANCODER_ID;
    this.angleFalconConfiguration.Feedback.SensorToMechanismRatio = 1;
    this.angleFalconConfiguration.Feedback.RotorToSensorRatio = Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO;
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
    this.flywheelFalconConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.flywheelFalconMaster.getConfigurator().apply(this.flywheelFalconConfiguration);
    this.flywheelFalconMaster.setNeutralMode(NeutralModeValue.Coast);
    this.flywheelFalconFollower.getConfigurator().apply(this.flywheelFalconConfiguration);
    this.flywheelFalconFollower.setNeutralMode(NeutralModeValue.Coast);

    this.flywheelFalconFollower.setControl(new Follower(Constants.CANInfo.SHOOTER_FLYWHEEL_MASTER_MOTOR_ID, true));
  }

  //Set shooter state, elevationAngle in rotations, flywheelVelocity in RPM
  public void set(double degrees, double RPM){
    double motionProfileScalar = (1.0 - this.angleFalconProfileScalarFactor) * Math.cos(Math.toRadians(getAngleDegrees())) + this.angleFalconProfileScalarFactor;
    // System.out.println("Scalar: " + motionProfileScalar);
    if (degrees > Constants.SetPoints.SHOOTER_MAX_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionMotionProfileRequest.withPosition(Constants.SetPoints.SHOOTER_MAX_ANGLE_ROT + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT).withAcceleration(this.angleFalconAcceleration * motionProfileScalar).withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    } else if (degrees < Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionMotionProfileRequest.withPosition(Constants.SetPoints.SHOOTER_DOWN_ANGLE_ROT + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT).withAcceleration(this.angleFalconAcceleration * motionProfileScalar).withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    } else {
      this.angleFalcon.setControl(this.angleFalconPositionMotionProfileRequest.withPosition(Constants.degreesToRotations(degrees) + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT).withAcceleration(this.angleFalconAcceleration * motionProfileScalar).withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    }
    this.flywheelFalconMaster.setControl(this.flywheelVelocityRequest.withVelocity(Constants.RPMToRPS(RPM) * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO));
  }

  //Set shooter angle in rotations
  public void setAngle(double degrees){
    double motionProfileScalar = (1 - this.angleFalconProfileScalarFactor) * Math.cos(Math.toRadians(getAngleDegrees())) + this.angleFalconProfileScalarFactor;
    if (degrees > Constants.SetPoints.SHOOTER_MAX_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionMotionProfileRequest.withPosition(Constants.SetPoints.SHOOTER_MAX_ANGLE_ROT + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT).withAcceleration(this.angleFalconAcceleration * motionProfileScalar).withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    } else if (degrees < Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionMotionProfileRequest.withPosition(Constants.SetPoints.SHOOTER_DOWN_ANGLE_ROT + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT).withAcceleration(this.angleFalconAcceleration * motionProfileScalar).withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    } else {
      this.angleFalcon.setControl(this.angleFalconPositionMotionProfileRequest.withPosition(Constants.degreesToRotations(degrees) + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT).withAcceleration(this.angleFalconAcceleration * motionProfileScalar).withJerk(this.angleFalconJerk * motionProfileScalar + Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT));
    }
  }

  //Set flywheel velocity in RPM
  public void setFlywheelRPM(double RPM){
    this.flywheelFalconMaster.setControl(this.flywheelVelocityRequest.withVelocity(Constants.RPMToRPS(RPM) * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO));
  }

  //Set flywheel percent
  public void setFlywheelPercent(double percent){
    this.flywheelFalconMaster.set(percent);
  }

  public void setFlywheelTorque(double current, double maxPercent){
    this.flywheelFalconMaster.setControl(this.flywheelTorqueRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public void setAnglePercent(double percent){
    this.angleFalcon.set(percent);
  }

  public double getFlywheelRPM(){
    return Constants.RPSToRPM(this.flywheelFalconMaster.getVelocity().getValueAsDouble() / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO);
  }

  public double getFlywheelFollowerRPM(){
    return Constants.RPSToRPM(this.flywheelFalconFollower.getVelocity().getValueAsDouble() / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO);
  }

  public double getAngleRotations(){
    return this.angleFalcon.getPosition().getValueAsDouble() - Constants.SetPoints.SHOOTER_CENTER_OFFSET_ROT;
  }

  public double getAngleDegrees(){
    return Constants.rotationsToDegrees(getAngleRotations());
  }

  //Constantly set flywheel velocity PID
  public void teleopPeriodic(){
    SmartDashboard.putNumber("Flywheel RPM", getFlywheelRPM());
    SmartDashboard.putNumber("Flywheel %", this.flywheelFalconMaster.getTorqueCurrent().getValueAsDouble());
    
    // SmartDashboard.putNumber("Flywheel %Out Master", this.flywheelVortexMaster.getAppliedOutput());
    // SmartDashboard.putNumber("Flywheel %Out Follower", this.flywheelVortexFollower.getAppliedOutput());
    // SmartDashboard.putNumber("Shooter Angle %", this.angleFalcon.getClosedLoopOutput().getValueAsDouble());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Angle Deg", getAngleDegrees());
  }
}
