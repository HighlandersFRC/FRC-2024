package frc.robot.subsystems;

//import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.defaults.IntakeDefault;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Feeder.FeederState;

public class Intake extends SubsystemBase {

  private final TalonFX angleFalcon = new TalonFX(Constants.CANInfo.INTAKE_ANGLE_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration angleFalconConfiguration = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC anglefalconPositionRequest = new PositionTorqueCurrentFOC(0, 0, 0, 0, false,
      false, false);
  private final TorqueCurrentFOC angleFalconCurrentRequest = new TorqueCurrentFOC(0, 0, 0, false, false, false);

  private final TalonFX rollerFalcon = new TalonFX(Constants.CANInfo.INTAKE_ROLLER_MOTOR_ID,
      Constants.CANInfo.CANBUS_NAME);
  private final TalonFXConfiguration rollerFalconConfiguration = new TalonFXConfiguration();
  private final VelocityTorqueCurrentFOC rollerFalconVelocityRequest = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false,
      false, false);

  public enum IntakeState {
    COLLECT,
    REJECT,
    EJECT,
    OFF,
    AMPTRAP,
    INDEX_TO_AMP,
    INDEX_TO_TRAP
  }

  double timeToCenterNote = 0.7;
  double runbackTime = 0.2;
  double haveNoteTime = 0.0;
  boolean haveNote = false;
  boolean noteInPlace = false;
  boolean noteInMiddle = false;

  boolean initRun = false;

  private IntakeState wantedState = IntakeState.OFF;
  private IntakeState systemState = IntakeState.OFF;

  /**
   * Constructs a new instance of the Intake class.
   */
  public Intake() {
    // setDefaultCommand(new IntakeDefault(this));
  }

  public void setWantedState(IntakeState wantedState) {
    this.wantedState = wantedState;
  }

  /**
   * Checks if intake CAN is available.
   *
   * @return {@code true} if the intake CAN is available, {@code false} otherwise.
   */
  public boolean getIntakeCAN() {
    if (angleFalcon.clearStickyFault_BootDuringEnable() == StatusCode.OK
        && rollerFalcon.clearStickyFault_BootDuringEnable() == StatusCode.OK) {
      return true;
    } else
      return false;
  }

  public void init() {
    this.angleFalconConfiguration.Slot0.kP = 40;
    this.angleFalconConfiguration.Slot0.kI = 0;
    this.angleFalconConfiguration.Slot0.kD = 6;
    this.angleFalconConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    this.angleFalconConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    this.angleFalconConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    this.angleFalconConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
    this.angleFalconConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
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

  public void autoInit() {
    this.angleFalcon.setPosition(-0.08);
  }

  /**
   * Sets the intake angle and roller velocity.
   *
   * @param degrees The desired angle of the intake in degrees.
   * @param RPM     The desired velocity of the roller in RPM (Revolutions Per
   *                Minute).
   */
  public void set(double degrees, double RPM) {
    if (degrees < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG) {
      // System.out.println("1");
      this.angleFalcon.setControl(this.anglefalconPositionRequest
          .withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (degrees > Constants.SetPoints.INTAKE_UP_ANGLE_DEG) {
      // System.out.println("2");
      this.angleFalcon.setControl(this.anglefalconPositionRequest
          .withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      // System.out.println("3");
      this.angleFalcon.setControl(this.anglefalconPositionRequest
          .withPosition(Constants.degreesToRotations(degrees) * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
    // System.out.println("Degrees: " + degrees);
    this.rollerFalcon.setControl(this.rollerFalconVelocityRequest
        .withVelocity(Constants.RPMToRPS(-RPM) * Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO));
  }

  /**
   * Sets the intake state and position along with the roller velocity.
   *
   * @param position The desired position for the intake (either kUP or kDOWN).
   * @param RPM      The desired velocity of the roller in RPM (Revolutions Per
   *                 Minute).
   */
  public void set(Constants.SetPoints.IntakePosition position, double RPM) {
    if (position.degrees < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG) {
      this.angleFalcon.setControl(this.anglefalconPositionRequest
          .withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (position.degrees > Constants.SetPoints.INTAKE_UP_ANGLE_DEG) {
      this.angleFalcon.setControl(this.anglefalconPositionRequest
          .withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      this.angleFalcon.setControl(
          this.anglefalconPositionRequest.withPosition(position.rotations * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
    this.rollerFalcon.setControl(this.rollerFalconVelocityRequest
        .withVelocity(Constants.RPMToRPS(-RPM) * Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO));
  }

  /**
   * Sets the intake angle with the specified setpoint.
   *
   * @param position The desired setpoint for the intake angle in degrees.
   */
  public void setAngle(double degrees) {
    if (degrees < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG) {
      this.angleFalcon.setControl(this.anglefalconPositionRequest
          .withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (degrees > Constants.SetPoints.INTAKE_UP_ANGLE_DEG) {
      this.angleFalcon.setControl(this.anglefalconPositionRequest
          .withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      this.angleFalcon.setControl(this.anglefalconPositionRequest
          .withPosition(Constants.degreesToRotations(degrees) * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
  }

  /**
   * Sets the intake angle with the specified setpoint.
   *
   * @param position The desired setpoint for the intake angle.
   */
  public void setAngle(Constants.SetPoints.IntakePosition position) {
    if (position.degrees < Constants.SetPoints.INTAKE_DOWN_ANGLE_DEG) {
      this.angleFalcon.setControl(this.anglefalconPositionRequest
          .withPosition(Constants.SetPoints.INTAKE_DOWN_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else if (position.degrees > Constants.SetPoints.INTAKE_UP_ANGLE_DEG) {
      this.angleFalcon.setControl(this.anglefalconPositionRequest
          .withPosition(Constants.SetPoints.INTAKE_UP_ANGLE_ROT * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    } else {
      this.angleFalcon.setControl(
          this.anglefalconPositionRequest.withPosition(position.rotations * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO));
    }
  }

  /**
   * Sets the velocity of the rollers.
   *
   * @param RPM The desired velocity of the rollers in RPM (Revolutions Per
   *            Minute).
   */
  public void setRollers(double RPM) {
    this.rollerFalcon.setControl(this.rollerFalconVelocityRequest
        .withVelocity(Constants.RPMToRPS(-RPM) * Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO));
  }

  /**
   * Set the Angle to a percent
   * 
   * @param percent - Current drawn from motor (amps)
   */
  public void setRollerPercent(double percent) {
    this.rollerFalcon.set(-percent);
  }

  /**
   * Set the Angle to a percent
   * 
   * @param percent - Current drawn from motor (amps)
   */
  public void setAnglePercent(double percent) {
    this.angleFalcon.set(percent);
  }

  /**
   * Set the Angle of the intake to Torque Current Control
   * 
   * @param current    - Current drawn from motor (amps)
   * @param maxPercent - Maximum motor power (from 0 to 1)
   */
  public void setAngleTorqueCurrent(double current, double maxPercent) {
    this.angleFalcon.setControl(this.angleFalconCurrentRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  /**
   * Set the current angle of the encoder to a position
   * 
   * @param rotations - The position to set in rotations
   */
  public void setAngleEncoderPosition(double rotations) {
    this.angleFalcon.setPosition(rotations * Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO);
  }

  /**
   * Get the velocity of intake roller in RPM
   * 
   * @return The velocity of the roller in RPM
   */
  public double getRPM() {
    return this.rollerFalcon.getVelocity().getValue() / Constants.Ratios.INTAKE_ROLLER_GEAR_RATIO;
  }

  /**
   * Get the angle of the intake in rotations
   * 
   * @return The position in rotations of the intake
   */
  public double getAngleRotations() {
    return this.angleFalcon.getPosition().getValue() / Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO;
  }

  /**
   * Get the angle of the intake in degrees
   * 
   * @return The position in degrees of the intake
   */
  public double getAngleDegrees() {
    return Constants
        .rotationsToDegrees(this.angleFalcon.getPosition().getValue() / Constants.Ratios.INTAKE_ANGLE_GEAR_RATIO);
  }

  /**
   * Get the stator current of the angle motor (amps)
   * 
   * @return The stator current of the angle motor in amps
   */
  public double getAngleCurrent() {
    return this.angleFalcon.getStatorCurrent().getValue();
  }

  /**
   * Get the velocity of the intake's angle in RPS
   * 
   * @return The angular velocity of the intake in RPS
   */
  public double getAngleRPS() {
    return this.angleFalcon.getVelocity().getValueAsDouble();
  }

  public double getRollerCurrent() {
    return this.rollerFalcon.getStatorCurrent().getValueAsDouble();
  }

  // Constantly set roller velocity PID
  public void teleopPeriodic() {

  }

  public void defaultState() {
    timeToCenterNote = 0.7;
    runbackTime = 0.2;
    haveNoteTime = 0.0;
    haveNote = false;
    noteInPlace = false;
    noteInMiddle = false;
    boolean isZeroed = false;
    int numTimesOverCurrentLimit = 0;
    double initTime = 0;
    if (!initRun) {
      initTime = Timer.getFPGATimestamp();
      initRun = true;
    }
    if (OI.getOperatorLB()) {
      this.setRollers(0);
    } else {
      this.setRollers(250);
    }
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);

    if (Timer.getFPGATimestamp() - initTime < 0.6) {
      this.setAngle(Constants.SetPoints.IntakePosition.kUP.degrees);
    } else {
      if (Math.abs(this.getAngleRPS()) < 0.01 && !isZeroed) {
        this.setAngleTorqueCurrent(10, 0.1);
        this.setAngleEncoderPosition(0);
        numTimesOverCurrentLimit++;
      } else if (!isZeroed) {
        this.setAngleTorqueCurrent(45, 0.3);
      } else {
        this.setAngleTorqueCurrent(5, 0.1);
      }

      if (numTimesOverCurrentLimit > 3) {
        isZeroed = true;
        numTimesOverCurrentLimit = 0;
      }

      if (Math.abs(this.getAngleRotations()) > 0.05) {
        isZeroed = false;
      }
    }
  }

  private IntakeState handleStateTransition() {
    switch (wantedState) {
      case REJECT:
        return IntakeState.REJECT;
      case EJECT:
        return IntakeState.EJECT;
      case AMPTRAP:
        return IntakeState.AMPTRAP;
      case COLLECT:
        if ((!Proximity.getShooterProximity() && Proximity.getFeederProximity())) {
          return IntakeState.OFF;
        }
        return IntakeState.COLLECT;
      case OFF:
      default:
        return IntakeState.OFF;
    }
  }

  public void intakeState() {
    boolean buzzControllers = false;

    boolean noteInIntake = false;
    int numTimeNoteInIntake = 0;

    if (getRollerCurrent() > Constants.SetPoints.INTAKE_CURRENT_THRESHOLD) {
      numTimeNoteInIntake++;
    }
    if (numTimeNoteInIntake > Constants.SetPoints.INTAKE_CURRENT_NUM_TIMES_IN_A_ROW_THRESHOLD) {
      noteInIntake = true;
    }

    set(Constants.SetPoints.IntakePosition.kDOWN, 1200);

    if (buzzControllers) {
      if (noteInIntake) {
        OI.driverController.setRumble(RumbleType.kBothRumble, 1.0);
        OI.operatorController.setRumble(RumbleType.kBothRumble, 1.0);
      }
    }
  }

  public void ampTrapState() {
    if (!(noteInPlace && Timer.getFPGATimestamp() - haveNoteTime > runbackTime)) {
      if (Proximity.getCarriageProximity() && !haveNote && !noteInPlace) {
        haveNoteTime = Timer.getFPGATimestamp();
        haveNote = true;
        // System.out.println("1");
      } else if (!Proximity.getCarriageProximity() && haveNote && !noteInPlace) {
        noteInMiddle = true;
        // System.out.println("2");
      } else if (Proximity.getCarriageProximity() && noteInMiddle && !noteInPlace) {
        noteInPlace = true;
        haveNoteTime = Timer.getFPGATimestamp();
        // System.out.println("3");
      } else if (noteInPlace) {
        // System.out.println("4");
      }
      this.setRollers(-150);
      this.setAngle(Constants.SetPoints.IntakePosition.kUP);
    } else {
      this.setAngleTorqueCurrent(40, 0.5);
      this.setRollers(60);
    }
  }

  @Override
  public void periodic() {
    // process inputs
    IntakeState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      systemState = IntakeState.OFF;
    }
    switch (systemState) {
      case COLLECT:
        intakeState();
        break;
      case REJECT:
        set(Constants.SetPoints.IntakePosition.kUP, -800);
        break;
      case EJECT:
        break;
      case AMPTRAP:
        ampTrapState();
        break;
      case OFF:
        defaultState();
        break;
      default:
        defaultState();
    }
 //   Logger.recordOutput("intakeRollerStator", this.rollerFalcon.getStatorCurrent().getValueAsDouble());
  }
}
