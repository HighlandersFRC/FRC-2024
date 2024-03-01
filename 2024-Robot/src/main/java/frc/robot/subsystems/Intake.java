package frc.robot.subsystems;

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

  public Intake() {
    setDefaultCommand(new IntakeDefault(this));
  }

  public void init(){
    this.angleFalconConfiguration.Slot0.kP = 1;
    this.angleFalconConfiguration.Slot0.kI = 0;
    this.angleFalconConfiguration.Slot0.kD = 0.1;
    this.angleFalconConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    this.angleFalconConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.angleFalconConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.angleFalconConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.angleFalconConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
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
    this.rollerFalcon.getConfigurator().apply(this.rollerFalconConfiguration);
    this.rollerFalcon.setNeutralMode(NeutralModeValue.Coast);
  }

  public void autoInit(){
    this.angleFalcon.setPosition(0);
  }

  public void set(double degrees, double RPM){
    if (degrees < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (degrees > Constants.SetPoints.INTAKE_UP_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.degreesToRotations(degrees) * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
    this.rollerFalcon.setControl(this.rollerFalconVelocityRequest.withVelocity(Constants.RPMToRPS(-RPM) * Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO));
  }

  //Set intake state, position either kUP or kDOWN, RPM in RPM
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

  //Set intake angle in rotations
  public void setAngle(double degrees){
    if (degrees < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (degrees > Constants.SetPoints.INTAKE_UP_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.degreesToRotations(degrees) * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
  }

  //Set intake angle with setpoint
  public void setAngle(Constants.SetPoints.IntakePosition position){
    if (position.degrees < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (position.degrees > Constants.SetPoints.INTAKE_UP_ANGLE_DEG){
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      this.angleFalcon.setControl(this.anglefalconPositionRequest.withPosition(position.rotations * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
  }

  //Set intake roller velocity in RPM
  public void setRollers(double RPM){
    this.rollerFalcon.setControl(this.rollerFalconVelocityRequest.withVelocity(Constants.RPMToRPS(-RPM) * Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO));
  }

  //Set intake roller percent
  public void setRollerPercent(double percent){
    this.rollerFalcon.set(-percent);
  }

  //Set intake angle motor percent
  public void setAnglePercent(double percent){
    this.angleFalcon.set(percent);
  }

  public void setAngleTorqueCurrent(double current, double maxPercent){
    this.angleFalcon.setControl(this.angleFalconCurrentRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public void setAngleEncoderPosition(double rotations){
    this.angleFalcon.setPosition(rotations * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO);
  }

  public double getRPM(){
    return this.rollerFalcon.getVelocity().getValue() / Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO;
  }

  public double getAngleRotations(){
    return this.angleFalcon.getPosition().getValue() / Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO;
  }

  public double getAngleDegrees(){
    return Constants.rotationsToDegrees(this.angleFalcon.getPosition().getValue() / Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO);
  }

  public double getAngleCurrent(){
    return this.angleFalcon.getStatorCurrent().getValue();
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
    boolean angleMotor = false;
    boolean intakeMotor = false;
    boolean intakeTOF = false;
    if(angleFalcon.getMotorVoltage().getValue() != 0.0 ){
      angleMotor = true;
    }
    if(rollerFalcon.getMotorVoltage().getValue() != 0.0){
      intakeMotor = true;
    }
    if(TOF.intakeTOF.getRange() > 0 && TOF.intakeTOF.getRange() < 1000.0){
      intakeTOF = true;
    }
    SmartDashboard.putBoolean(" Intake Angle Motor", angleMotor);
    SmartDashboard.putBoolean(" Intake Roller Motor", intakeMotor);
    SmartDashboard.putBoolean(" Intake TOF", intakeTOF);
  }
}
