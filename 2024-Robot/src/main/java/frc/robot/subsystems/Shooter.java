package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
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
  private final PositionTorqueCurrentFOC angleFalconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  private final TalonFX flywheelFalconMaster = new TalonFX(Constants.CANInfo.SHOOTER_FLYWHEEL_MASTER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration flywheelFalconConfiguration = new TalonFXConfiguration();
  private final TalonFX flywheelFalconFollower = new TalonFX(Constants.CANInfo.SHOOTER_FLYWHEEL_FOLLOWER_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);

  public Shooter() {
    setDefaultCommand(new ShooterDefault(this));
  }

  public void init(){
    this.angleEncoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    this.angleEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    this.angleFalconConfiguration.Slot0.kP = 0;
    this.angleFalconConfiguration.Slot0.kI = 0;
    this.angleFalconConfiguration.Slot0.kD = 0;
    this.angleFalconConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    this.angleFalconConfiguration.Slot0.kG = 0;

    this.angleFalconConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    this.angleFalconConfiguration.Feedback.FeedbackRemoteSensorID = Constants.CANInfo.SHOOTER_ANGLE_CANCODER_ID;
    this.angleFalconConfiguration.Feedback.SensorToMechanismRatio = 1;
    this.angleFalconConfiguration.Feedback.RotorToSensorRatio = Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO;
    this.angleFalcon.getConfigurator().apply(this.angleFalconConfiguration);
    this.angleFalcon.setNeutralMode(NeutralModeValue.Brake);
    this.angleFalcon.setPosition(this.angleEncoder.getAbsolutePosition().getValue());
    this.angleFalcon.set(0);

    double flywheelP = 0.01;
    double flywheelI = 0.00000;
    double flywheelD = 0.0;
    double flywheelFF = 0.0002;

    this.flywheelFalconFollower.setControl(new Follower(Constants.CANInfo.SHOOTER_FLYWHEEL_MASTER_MOTOR_ID, true));
  }

  //Set shooter state, elevationAngle in rotations, flywheelVelocity in RPM
  public void set(double degrees, double RPM){
    if (degrees > Constants.SetPoints.SHOOTER_MAX_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.SetPoints.SHOOTER_MAX_ANGLE_ROT));
    } else if (degrees < Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.SetPoints.SHOOTER_DOWN_ANGLE_ROT));
    } else {
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.degreesToRotations(degrees)));
    }
  }

  //Set shooter angle in rotations
  public void setAngle(double degrees){
    if (degrees > Constants.SetPoints.SHOOTER_MAX_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.SetPoints.SHOOTER_MAX_ANGLE_ROT));
    } else if (degrees < Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.SetPoints.SHOOTER_DOWN_ANGLE_ROT));
    } else {
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.degreesToRotations(degrees)));
    }
  }

  //Set flywheel velocity in RPM
  public void setVelocity(double RPM){}

  //Set flywheel percent
  public void setFlywheelPercent(double percent){
    this.flywheelFalconMaster.set(percent);
  }

  public void setAnglePercent(double percent){
    this.angleFalcon.set(percent);
  }

  public double getFlywheelRPM(){
    return this.flywheelFalconMaster.getRotorVelocity().getValue() / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO;
  }

  public double getFlywheelFollowerRPM(){
    return this.flywheelFalconFollower.getRotorVelocity().getValue() / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO;
  }

  public double getAngleRotations(){
    return this.angleFalcon.getPosition().getValue();
  }

  public double getAngleDegrees(){
    return Constants.rotationsToDegrees(getAngleRotations());
  }

  //Constantly set flywheel velocity PID
  public void teleopPeriodic(){
    SmartDashboard.putNumber("Flywheel RPM", getFlywheelRPM());
    // SmartDashboard.putNumber("Flywheel Follower RPM", getFlywheelFollowerRPM());
    SmartDashboard.putNumber("Shooter Angle Deg", getAngleDegrees());
    // SmartDashboard.putNumber("Flywheel %Out Master", this.flywheelVortexMaster.getAppliedOutput());
    // SmartDashboard.putNumber("Flywheel %Out Follower", this.flywheelVortexFollower.getAppliedOutput());
  }

  @Override
  public void periodic() {

  }
}
