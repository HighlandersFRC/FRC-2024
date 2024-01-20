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
import frc.robot.commands.defaults.IntakeDefault;

public class Intake extends SubsystemBase {
  private Lights lights;

  private final TalonFX angleFalcon = new TalonFX(Constants.CANInfo.INTAKE_ANGLE_MOTOR_ID, Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration angleFalconConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC falconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false, false);

  private final CANSparkFlex rollerVortex = new CANSparkFlex(Constants.CANInfo.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);
  private final SparkAbsoluteEncoder rollerVortexEncoder = rollerVortex.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private final SparkPIDController rollerVortexPID = rollerVortex.getPIDController();
  private double rollerVortexVelocitySetpoint = 0;

  public Intake(Lights lights) {
    this.lights = lights;
    setDefaultCommand(new IntakeDefault(this));
  }

  public void init(){
    this.rollerVortexVelocitySetpoint = 0;

    this.angleFalconConfiguration.Slot0.kP = 0;
    this.angleFalconConfiguration.Slot0.kI = 0;
    this.angleFalconConfiguration.Slot0.kD = 0;
    this.angleFalcon.getConfigurator().apply(this.angleFalconConfiguration);

    this.rollerVortexPID.setP(0, 0);
    this.rollerVortexPID.setI(0, 0);
    this.rollerVortexPID.setD(0, 0);
    this.rollerVortexPID.setFF(0, 0);
    this.rollerVortexPID.setOutputRange(-1, 1);
    this.rollerVortexEncoder.setPositionConversionFactor(1);
    this.rollerVortexEncoder.setVelocityConversionFactor(1);
    this.rollerVortex.setIdleMode(IdleMode.kCoast);
  }

  //Set intake state, rotationAngle in rotations, rollerVelocity in RPM
  public void setIntake(double rotationAngle, double rollerVelocity){
    this.angleFalcon.setControl(this.falconPositionRequest.withPosition(rotationAngle));
    this.rollerVortexVelocitySetpoint = rollerVelocity;
  }

  //Set intake state, position either kUP or kDOWN, rollerVelocity in RPM
  public void setIntake(Constants.Setpoints.IntakePosition position, double rollerVelocity){
    this.angleFalcon.setControl(this.falconPositionRequest.withPosition(position.angle));
    this.rollerVortexVelocitySetpoint = rollerVelocity;
  }

  //Constantly set vortex pid
  public void teleopPeriodic(){
    this.rollerVortexPID.setReference(this.rollerVortexVelocitySetpoint, CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void periodic() {}

  
}
