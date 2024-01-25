package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
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
import frc.robot.commands.defaults.ShooterDefault;

public class Shooter extends SubsystemBase {
  private Lights lights;

  private final CANcoder angleEncoder = new CANcoder(Constants.CANInfo.SHOOTER_ANGLE_CANCODER_ID, Constants.CANInfo.CANBUS_NAME);

  private final TalonFX angleFalcon = new TalonFX(Constants.CANInfo.SHOOTER_ANGLE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration angleFalconConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC angleFalconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  private final CANSparkFlex flywheelVortexMaster = new CANSparkFlex(Constants.CANInfo.SHOOTER_FLYWHEEL_MASTER_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder flywheelVortexMasterEncoder = flywheelVortexMaster.getEncoder();
  private final SparkPIDController flywheelVortexMasterPID = flywheelVortexMaster.getPIDController();
  private final CANSparkFlex flywheelVortexFollower = new CANSparkFlex(Constants.CANInfo.SHOOTER_FLYWHEEL_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder flywheelVortexFollowerEncoder = flywheelVortexFollower.getEncoder();
  private final SparkPIDController flywheelVortexFollowerPID = flywheelVortexFollower.getPIDController();

  public Shooter(Lights lights) {
    this.lights = lights;
    setDefaultCommand(new ShooterDefault(this));
  }

  public void init(){
    this.angleFalconConfiguration.Slot0.kP = 600;
    this.angleFalconConfiguration.Slot0.kI = 0;
    this.angleFalconConfiguration.Slot0.kD = 0;
    this.angleFalconConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    this.angleFalconConfiguration.Slot0.kG = 10;
    this.angleFalconConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    this.angleFalconConfiguration.Feedback.FeedbackRemoteSensorID = Constants.CANInfo.SHOOTER_ANGLE_CANCODER_ID;
    this.angleFalconConfiguration.Feedback.SensorToMechanismRatio = 1;
    this.angleFalconConfiguration.Feedback.RotorToSensorRatio = Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO;
    this.angleFalcon.getConfigurator().apply(this.angleFalconConfiguration);
    this.angleFalcon.setNeutralMode(NeutralModeValue.Brake);
    this.angleFalcon.setPosition(this.angleEncoder.getAbsolutePosition().getValue());
    this.angleFalcon.set(0);

    double flywheelP = 0.005;
    double flywheelI = 0.000;
    double flywheelD = 0;
    double flywheelFF = 0;

    this.flywheelVortexMaster.restoreFactoryDefaults();
    this.flywheelVortexMasterPID.setP(flywheelP, 0);
    this.flywheelVortexMasterPID.setI(flywheelI, 0);
    this.flywheelVortexMasterPID.setD(flywheelD, 0);
    this.flywheelVortexMasterPID.setFF(flywheelFF, 0);
    this.flywheelVortexMasterPID.setOutputRange(-1, 1);
    this.flywheelVortexMaster.setIdleMode(IdleMode.kCoast);
    this.flywheelVortexMaster.set(0);

    this.flywheelVortexFollower.restoreFactoryDefaults();
    this.flywheelVortexFollower.follow(this.flywheelVortexMaster, true);
    this.flywheelVortexFollowerPID.setP(flywheelP, 0);
    this.flywheelVortexFollowerPID.setI(flywheelI, 0);
    this.flywheelVortexFollowerPID.setD(flywheelD, 0);
    this.flywheelVortexFollowerPID.setFF(flywheelFF, 0);
    this.flywheelVortexFollowerPID.setOutputRange(-1, 1);
    this.flywheelVortexFollower.setIdleMode(IdleMode.kCoast);
    this.flywheelVortexFollower.set(0);
  }

  //Set shooter state, elevationAngle in rotations, flywheelVelocity in RPM
  public void setShooter(double elevationAngle, double flywheelVelocity){
    if (elevationAngle > Constants.SetPoints.SHOOTER_MAX_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.SetPoints.SHOOTER_MAX_ANGLE_ROT));
    } else if (elevationAngle < Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.SetPoints.SHOOTER_DOWN_ANGLE_ROT));
    } else {
      System.out.println("4 " + elevationAngle);
      System.out.println("pos " + getShooterAngle());
      System.out.println("final " + Constants.degreesToRotations(elevationAngle));
      System.out.println("power " + this.angleFalcon.getAppliedControl().getControlInfo().toString());
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.degreesToRotations(elevationAngle)));
    }
    this.flywheelVortexMasterPID.setReference(flywheelVelocity * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO, CANSparkBase.ControlType.kVelocity);
  }

  //Set shooter angle in rotations
  public void setShooterAngle(double elevationAngle){
    if (elevationAngle > Constants.SetPoints.SHOOTER_MAX_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.SetPoints.SHOOTER_MAX_ANGLE_ROT));
    } else if (elevationAngle < Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.SetPoints.SHOOTER_DOWN_ANGLE_ROT));
    } else {
      this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(Constants.degreesToRotations(elevationAngle)));
    }
  }

  //Set flywheel velocity in RPM
  public void setShooterVelocity(double flywheelVelocity){
    this.flywheelVortexMasterPID.setReference(flywheelVelocity * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO, CANSparkBase.ControlType.kVelocity);
  }

  //Set flywheel percent
  public void setShooterPercent(double percent){
    this.flywheelVortexMaster.set(percent);
  }

  public double getFlywheelRPM(){
    return this.flywheelVortexMasterEncoder.getVelocity() / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO;
  }

  public double getShooterAngle(){
    return this.angleFalcon.getPosition().getValue();
  }

  //Constantly set flywheel velocity PID
  public void teleopPeriodic(){
    SmartDashboard.putNumber("Flywheel RPM", getFlywheelRPM());
    SmartDashboard.putNumber("Shooter Elevation", getShooterAngle());
    // SmartDashboard.putNumber("Shooter elev falcon" )
  }

  @Override
  public void periodic() {

  }
}
