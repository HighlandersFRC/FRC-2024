package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final TalonFX rotationFalcon = new TalonFX(10, "Canivore");
  private final TalonFXConfiguration rotationFalconConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC falconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  private final CANSparkFlex rollerVortex = new CANSparkFlex(11, MotorType.kBrushless);
  private final SparkAbsoluteEncoder rollerVortexEncoder = rollerVortex.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private final SparkPIDController rollerVortexPID = rollerVortex.getPIDController();
  private double rollerVortexVelocitySetpoint = 0;

  public Intake() {

  }

  public void init(){
    this.rollerVortexVelocitySetpoint = 0;

    this.rotationFalconConfiguration.Slot0.kP = 0;
    this.rotationFalconConfiguration.Slot0.kI = 0;
    this.rotationFalconConfiguration.Slot0.kD = 0;
    this.rotationFalcon.getConfigurator().apply(this.rotationFalconConfiguration);

    this.rollerVortexPID.setP(0, 0);
    this.rollerVortexPID.setI(0, 0);
    this.rollerVortexPID.setD(0, 0);
    this.rollerVortexPID.setFF(0, 0);
    this.rollerVortexPID.setOutputRange(-1, 1);
    this.rollerVortexPID.setSmartMotionMaxVelocity(1000, 0);
    this.rollerVortexPID.setSmartMotionMinOutputVelocity(-1000, 0);
    this.rollerVortexPID.setSmartMotionMaxAccel(100, 0);
    this.rollerVortexEncoder.setPositionConversionFactor(1);
    this.rollerVortexEncoder.setVelocityConversionFactor(1);
    this.rollerVortex.setIdleMode(IdleMode.kCoast);
  }

  //Set intake state, rotationAngle in rotations, rollerVelocity in RPM
  public void setIntake(double rotationAngle, double rollerVelocity){
    this.rotationFalcon.setControl(this.falconPositionRequest.withPosition(rotationAngle));
    this.rollerVortexVelocitySetpoint = rollerVelocity;
  }

  //Set intake state, position either kUP or kDOWN, rollerVelocity in RPM
  public void setIntake(Constants.Setpoints.IntakePosition position, double rollerVelocity){
    this.rotationFalcon.setControl(this.falconPositionRequest.withPosition(position.angle));
    this.rollerVortexVelocitySetpoint = rollerVelocity;
  }

  //Constantly set vortex pid
  public void teleopPeriodic(){
    this.rollerVortexPID.setReference(this.rollerVortexVelocitySetpoint, CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void periodic() {}

  
}
