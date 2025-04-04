// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;
//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoPositionalShoot;
import frc.robot.commands.AutoPrepForShot;
import frc.robot.commands.DipShot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.DriveAutoAligned;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.PositionalDipShot;
import frc.robot.commands.PositionalFeederLobShot;
import frc.robot.commands.PositionalLobShot;
import frc.robot.commands.PositionalSpinUp;
import frc.robot.commands.PresetAutoShoot;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunFlywheel;
import frc.robot.commands.RunIntakeAndFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SetRobotState;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.commands.presets.AmpPreset;
import frc.robot.commands.presets.TrapPreset;
import frc.robot.sensors.Proximity;
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Sensors
  TOF tof = new TOF();
  Proximity proximity = new Proximity();

  // Subsystems
  Lights lights = new Lights(tof);
  Peripherals peripherals = new Peripherals();
  Drive drive = new Drive(peripherals);
  Intake intake = new Intake();
  Shooter shooter = new Shooter();
  Feeder feeder = new Feeder(tof, proximity, () -> getNoteInRobot());
  Climber climber = new Climber(lights, tof, proximity);
  Superstructure superstructure = new Superstructure(drive, intake, shooter, feeder, lights, peripherals, climber,
      proximity);

  HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
    {
      put("Instant", () -> new InstantCommand());
      put("Intake", () -> new AutoIntake(intake, feeder, climber, lights, tof, proximity,
          Constants.SetPoints.IntakePosition.kDOWN, 1200, 400, false, false));
      put("Outtake", () -> new RunIntakeAndFeeder(intake, feeder, climber, Constants.SetPoints.IntakePosition.kUP, -800,
          -800, -0.4));
      put("Shoot",
          () -> new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 22, 7000, true));
      put("Auto Spin Up", () -> new PositionalSpinUp(drive, shooter, peripherals, lights, proximity));
      put("Spin Up No Note", () -> new RunShooter(shooter, Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG, 5000));
      put("Wait", () -> new DoNothing());
      put("Subwoofer Preset",
          () -> new PresetAutoShoot(drive, shooter, feeder, peripherals, lights, proximity, 58, 4900, 1200, 0));
    }
  };

  File[] autoFiles = new File[Constants.Autonomous.paths.length];
  Command[] autos = new Command[Constants.Autonomous.paths.length];
  JSONObject[] autoJSONs = new JSONObject[Constants.Autonomous.paths.length];
  JSONArray[] autoPoints = new JSONArray[Constants.Autonomous.paths.length];
  int timesNoteSeen = 0;
  int TOFfilterThreshold = 2;

  private int numTimeNoteInIntake = 0;
  private double intakeTime = 0;

  /**
   * This function checks if a note is currently detected in the intake.
   * 
   * The function compares the current count of times a note is detected in the
   * intake
   * with a predefined threshold. If the count exceeds the threshold, the function
   * returns true. Additionally, the function checks if the time elapsed since the
   * last
   * note was detected in the intake is within a predefined threshold. If the time
   * elapsed is within the threshold, the function returns true. Otherwise, the
   * function returns false.
   * 
   * @return A boolean value indicating whether a note is currently detected in
   *         the intake.
   *         The function returns true if a note is detected, and false otherwise.
   */
  boolean getNoteInIntake() {
    boolean retval = (this.numTimeNoteInIntake > Constants.SetPoints.INTAKE_CURRENT_NUM_TIMES_IN_A_ROW_THRESHOLD);
    if (retval)
      intakeTime = Timer.getFPGATimestamp();
    if (Timer.getFPGATimestamp() - intakeTime < Constants.SetPoints.TIME_EXTENSION_INTAKE_THRESHOLD)
      retval = true;
    //Logger.recordOutput("Note in Intake", retval);
    return retval;
  }

  /**
   * This function updates the count of times a note is detected in the intake.
   * 
   * The function checks the current of the intake roller. If the current exceeds
   * a
   * predefined threshold, it increments the count of times a note is detected. If
   * the
   * current is below the threshold, it resets the count to zero.
   * 
   * @return void
   */
  void updateNoteInIntake() {
    if (this.intake.getRollerCurrent() > Constants.SetPoints.INTAKE_CURRENT_THRESHOLD) {
      this.numTimeNoteInIntake++;
    } else {
      this.numTimeNoteInIntake = 0;
    }
  }

  /**
   * This function checks if there is a note in the robot.
   * 
   * @return A boolean value indicating whether there is a note in the robot.
   *         The function returns true if there is a note in the robot (either in
   *         the intake,
   *         feeder proximity, or shooter proximity), and false otherwise.
   */
  boolean getNoteInRobot() {
    boolean retval = getNoteInIntake() || proximity.getFeederProximity()
        || proximity.getShooterProximity();
    return retval;
  }

  HashMap<String, BooleanSupplier> conditionMap = new HashMap<String, BooleanSupplier>() {
    {
      put("Note In Intake", () -> getNoteInRobot());
    }
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Load the Path Files
    for (int i = 0; i < Constants.Autonomous.paths.length; i++) {
      try {
        autoFiles[i] = new File(Filesystem.getDeployDirectory().getPath() + "/" + Constants.Autonomous.paths[i]);
        FileReader scanner = new FileReader(autoFiles[i]);
        autoJSONs[i] = new JSONObject(new JSONTokener(scanner));
        autoPoints[i] = (JSONArray) autoJSONs[i].getJSONArray("paths").getJSONObject(0).getJSONArray("sampled_points");
        autos[i] = new PolarAutoFollower(autoJSONs[i], drive, lights, peripherals, commandMap, conditionMap);
      } catch (Exception e) {
        System.out.println("ERROR LOADING PATH " + Constants.Autonomous.paths[i] + ":" + e);
      }
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // COMPETITION CONTROLS
    // Driver

    OI.driverViewButton.whileTrue(new ZeroAngleMidMatch(drive));
    // OI.driverB.whileTrue(new PositionalLobShot(drive, shooter, feeder,
    // peripherals, lights, proximity, 1200, 2)); // tests
    // OI.driverRT.whileTrue(new AutoIntake(intake, feeder, climber, lights, tof,
    // proximity,
    // Constants.SetPoints.IntakePosition.kDOWN, 1200, 450, true, true));
    // OI.driverLT.whileTrue(
    // new RunIntakeAndFeeder(intake, feeder, climber,
    // Constants.SetPoints.IntakePosition.kUP, -800, -800, -0.4));
    // OI.operatorLB.whileTrue(new LobShot(drive, shooter, feeder, peripherals,
    // lights, proximity, 55, 4400, 1200, 0, 193, 149, 5));
    // OI.driverA.whileTrue(
    // new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights,
    // proximity, 1200, 22, 7000, false));
     OI.driverRT.whileTrue(new SetRobotState(superstructure, SuperState.INTAKE));
     OI.driverLT.whileTrue(new SetRobotState(superstructure, SuperState.OUTAKING));
    OI.driverA.whileTrue(new SetRobotState(superstructure, SuperState.LOW_SHOT));
     OI.driverB.whileTrue(new SetRobotState(superstructure, SuperState.HIGH_SHOT));
    OI.driverY.whileTrue(new SetRobotState(superstructure, SuperState.HIGH_SLOW_SHOT));
     // OI.driverA.onFalse(superstructure.setWantedSuperStateCommand(SuperState.CYCLING));
    // OI.driverX.whileTrue(new DriveAutoAligned(drive, peripherals));
    // OI.driverPOVDown]\[]
    // .whileTrue(new PresetAutoShoot(drive, shooter, feeder, peripherals, lights,
    // proximity, 60, 4500, 1200, 0, 1.5));
    // OI.driverPOVLeft
    // .whileTrue(new DipShot(drive, shooter, feeder, peripherals, lights,
    // proximity, 10, 6200, 1200, 0, 0, 0, 5));

    // Operator
    // OI.operatorA.whileTrue(new SetRobotState(superstructure, SuperState.CLIMBER_DOWN));
    // OI.operatorY.whileTrue(new SetRobotState(superstructure, SuperState.CLIMBER_UP));
    // OI.operatorB.whileTrue(new SetRobotState(superstructure, SuperState.TRAP));
     OI.driverX.whileTrue(new SetRobotState(superstructure, SuperState.AMP));
    // OI.operatorX.whileTrue(new AmpPreset(climber, feeder, intake, proximity,
    // shooter));
    // OI.operatorB.whileTrue(new TrapPreset(climber, feeder, intake, proximity,
    // shooter));
    // OI.operatorY.whileTrue(new RunClimber(climber, feeder, 20, 1.0));
    // OI.operatorA.whileTrue(new RunClimber(climber, feeder, -50, 1.0));
    // OI.operatorRT.whileTrue(new AutoPrepForShot(shooter, proximity, 55, 4600));
    // // OI.operatorRB.whileTrue(new SmartPrepForShot(shooter, peripherals,
    // lights));
    // OI.operatorRB.whileTrue(new PositionalSpinUp(drive, shooter, peripherals,
    // lights, proximity));
    // OI.operatorMenuButton.whileTrue(new RunFlywheel(shooter, 80, 0.2));
    // OI.operatorViewButton
    // .whileTrue(new AutoShoot(drive, shooter, feeder, peripherals, lights,
    // proximity, 1200, 22, 7000, false));
    // OI.operatorLB.whileTrue(new PositionalLobShot(drive, shooter, feeder,
    // peripherals, lights, proximity, 1200, 5));
    // OI.operatorLJ
    // .whileTrue(new PositionalFeederLobShot(drive, shooter, feeder, peripherals,
    // lights, proximity, 1200, 5));
    // OI.operatorRJ
    // .whileTrue(new PositionalDipShot(drive, shooter, feeder, peripherals, lights,
    // proximity, 5, 6200, 1200, 0, 5));
    // // OI.operatorRB.whileTrue(new AutoIntake(intake, feeder, climber, lights,
    // tof,
    // // Constants.SetPoints.IntakePosition.kDOWN, 1200, 400));
    // OI.operatorLT.whileTrue(new AutoIntake(intake, feeder, climber, lights, tof,
    // proximity,
    // Constants.SetPoints.IntakePosition.kDOWN, 1200, 450, true, true));
    // OI.operatorViewButton.whileTrue(new RunFeeder(feeder, -300));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    final int selectedPath = Constants.Autonomous.getSelectedPathIndex();
    if (selectedPath == -1) {
      System.out.println("Do Nothing");
      return new DoNothing();
    } else {
      this.drive.autoInit(autoPoints[selectedPath]);
      System.out.println(Constants.Autonomous.paths[selectedPath]);
      return this.autos[selectedPath];
    }
  }
}
