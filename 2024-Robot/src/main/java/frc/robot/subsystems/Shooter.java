package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ShooterDefault;

public class Shooter extends SubsystemBase {
  private Lights lights;

  private final TalonFX angleFalcon = new TalonFX(Constants.CANInfo.SHOOTER_ANGLE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration angleFalconConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC angleFalconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  private final CANSparkFlex flywheelVortexMaster = new CANSparkFlex(Constants.CANInfo.SHOOTER_FLYWHEEL_MASTER_MOTOR_ID, MotorType.kBrushless);
  private final SparkAbsoluteEncoder flywheelVortexMasterEncoder = flywheelVortexMaster.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController flywheelVortexMasterPID = flywheelVortexMaster.getPIDController();
  private final CANSparkFlex flywheelVortexFollower = new CANSparkFlex(Constants.CANInfo.SHOOTER_FLYWHEEL_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
  private final SparkAbsoluteEncoder flywheelVortexFollowerEncoder = flywheelVortexFollower.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController flywheelVortexFollowerPID = flywheelVortexFollower.getPIDController();

  private double flywheelVelocitySetpoint = 0;

  public Shooter(Lights lights) {
    this.lights = lights;
    setDefaultCommand(new ShooterDefault(this));
  }

  public void init(){
    this.flywheelVelocitySetpoint = 0;

    this.angleFalconConfiguration.Slot0.kP = 0;
    this.angleFalconConfiguration.Slot0.kI = 0;
    this.angleFalconConfiguration.Slot0.kD = 0;
    this.angleFalcon.getConfigurator().apply(this.angleFalconConfiguration);

    this.flywheelVortexMaster.restoreFactoryDefaults();
    this.flywheelVortexMasterPID.setP(0, 0);
    this.flywheelVortexMasterPID.setI(0, 0);
    this.flywheelVortexMasterPID.setD(0, 0);
    this.flywheelVortexMasterPID.setFF(0, 0);
    this.flywheelVortexMasterPID.setOutputRange(-1, 1);
    this.flywheelVortexMasterEncoder.setPositionConversionFactor(1);
    this.flywheelVortexMasterEncoder.setVelocityConversionFactor(1);
    this.flywheelVortexMaster.setIdleMode(IdleMode.kCoast);

    this.flywheelVortexFollower.restoreFactoryDefaults();
    this.flywheelVortexFollower.follow(this.flywheelVortexMaster);
    this.flywheelVortexFollowerPID.setP(0, 0);
    this.flywheelVortexFollowerPID.setI(0, 0);
    this.flywheelVortexFollowerPID.setD(0, 0);
    this.flywheelVortexFollowerPID.setFF(0, 0);
    this.flywheelVortexFollowerPID.setOutputRange(-1, 1);
    this.flywheelVortexFollowerEncoder.setPositionConversionFactor(1);
    this.flywheelVortexFollowerEncoder.setVelocityConversionFactor(1);
    this.flywheelVortexFollower.setIdleMode(IdleMode.kCoast);
  }

  //Set shooter state, elevationAngle in rotations, flywheelVelocity in RPM
  public void setShooter(double elevationAngle, double flywheelVelocity){
    this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(elevationAngle * Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO));
    this.flywheelVelocitySetpoint = flywheelVelocity;
  }

  //Set shooter angle in rotations
  public void setShooterAngle(double elevationAngle){
    this.angleFalcon.setControl(this.angleFalconPositionRequest.withPosition(elevationAngle * Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO));
  }

  //Set flywheel velocity in RPM
  public void setShooterVelocity(double flywheelVelocity){
    this.flywheelVelocitySetpoint = flywheelVelocity;
  }

  //Constantly set flywheel velocity PID
  public void teleopPeriodic(){
    this.flywheelVortexMasterPID.setReference(this.flywheelVelocitySetpoint * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO, CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void periodic() {}
}
