package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoPositionalShoot;
import frc.robot.commands.AutoPrepForShot;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.DipShot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.DriveAutoAligned;
import frc.robot.commands.LobShot;
import frc.robot.commands.PositionalDipShot;
import frc.robot.commands.PositionalFeederLobShot;
import frc.robot.commands.PositionalLobShot;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.PositionalSpinUp;
import frc.robot.commands.PresetAutoShoot;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunFlywheel;
import frc.robot.commands.RunIntakeAndFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.commands.autos.FivePiece3Auto;
import frc.robot.commands.autos.FourPieceFarBottom231Auto;
import frc.robot.commands.autos.NothingAuto;
import frc.robot.commands.autos.StraightLine;
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

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  // Sensors
  private TOF tof = new TOF();
  private Proximity proximity = new Proximity();

  // Subsystems
  private Lights lights = new Lights(tof);
  private Peripherals peripherals = new Peripherals();
  private Drive drive = new Drive(peripherals);
  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private Feeder feeder = new Feeder(tof, proximity, () -> getNoteInRobot());
  private Climber climber = new Climber(lights, tof, proximity);

  // private Logger logger = Logger.getInstance();

  private double shooterAngleDegreesTuning = 0;
  private double shooterRPMTuning = 0;
  private double startTime = Timer.getFPGATimestamp();
  private boolean checkedCAN = false;
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
      put("Subwoofer Preset", () -> new PresetAutoShoot(drive, shooter, feeder, peripherals, lights, proximity, 58, 4900, 1200, 0));
    }
  };
  int timesNoteSeen = 0;
  int TOFfilterThreshold = 2;

  private int numTimeNoteInIntake = 0;
  private double intakeTime = 0;

  boolean getNoteInIntake() {
    boolean retval = (this.numTimeNoteInIntake > Constants.SetPoints.INTAKE_CURRENT_NUM_TIMES_IN_A_ROW_THRESHOLD);
    if (retval) intakeTime = Timer.getFPGATimestamp();
    if (Timer.getFPGATimestamp()-intakeTime<Constants.SetPoints.TIME_EXTENSION_INTAKE_THRESHOLD) retval = true;
    Logger.recordOutput("Note in Intake", retval);
    return retval;
  }
  void updateNoteInIntake() {
    if (this.intake.getRollerCurrent() > Constants.SetPoints.INTAKE_CURRENT_THRESHOLD){
      this.numTimeNoteInIntake++;
    } else {
      this.numTimeNoteInIntake = 0;
    }
  }

  boolean getNoteInRobot() {
    boolean retval = getNoteInIntake() || proximity.getFeederProximity()
        || proximity.getShooterProximity() ;
    return retval;
  }

  HashMap<String, BooleanSupplier> conditionMap = new HashMap<String, BooleanSupplier>() {
    {
      put("Note In Intake", () -> getNoteInRobot());
    }
  };

  Command nothingAuto;
  Command fivePiece3Note;
  Command fourPieceFarBottom231Auto;

  
  File[] autoFiles = new File[Constants.paths.length];
  Command[] autos = new Command[Constants.paths.length];
  JSONObject[] autoJSONs = new JSONObject[Constants.paths.length];
  JSONArray[] autoPoints = new JSONArray[Constants.paths.length];
  Command dipShot;

  String fieldSide = "blue";

  @Override
  public void robotInit() {
    System.out.println("Robot Init");
    // System.out.println("Starting");
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution
    // logging
    // Logger.recordMetadata("Code", "Running");
    // Logger.disableDeterministicTimestamps(); // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    // System.out.println("Started Logger");
    this.fieldSide = "blue";
    SmartDashboard.putNumber("Shooter Angle Degrees (tuning)", 0);
    SmartDashboard.putNumber("Shooter RPM (input)", 0);

    lights.init(fieldSide);
    peripherals.init();
    drive.init(fieldSide);
    intake.init();
    shooter.init();
    feeder.init();
    climber.init();

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);

    PortForwarder.add(5800, "limelight-front.local", 5800);
    PortForwarder.add(5801, "limelight-front.local", 5801);

    PortForwarder.add(5800, "limelight-back.local", 5800);
    PortForwarder.add(5801, "limelight-back.local", 5801);

    PortForwarder.add(5800, "limelight-left.local", 5800);
    PortForwarder.add(5801, "limelight-left.local", 5801);

    PortForwarder.add(5800, "limelight-right.local", 5800);
    PortForwarder.add(5801, "limelight-right.local", 5801);

    PortForwarder.add(5800, "10.44.99.41", 5800);
    PortForwarder.add(5801, "10.44.99.41", 5801);

    PortForwarder.add(5800, "10.44.99.42", 5800);
    PortForwarder.add(5801, "10.44.99.42", 5801);

    PortForwarder.add(5800, "10.44.99.43", 5800);
    PortForwarder.add(5801, "10.44.99.43", 5801);

    PortForwarder.add(5800, "10.44.99.44", 5800);
    PortForwarder.add(5801, "10.44.99.44", 5801);

    // System.out.println("ports forwarded");
    this.nothingAuto = new NothingAuto();
    // this.dipShot = new DipShot(drive, shooter, feeder, peripherals, lights,
    // proximity, 55, 5500, 1200, 0, 185, 150, 5);

    for (int i = 0; i < Constants.paths.length; i++){
      try {
        autoFiles[i] = new File(Filesystem.getDeployDirectory().getPath() + "/" + Constants.paths[i]);
        FileReader scanner = new FileReader(autoFiles[i]);
        autoJSONs[i] = new JSONObject(new JSONTokener(scanner));
        autoPoints[i] = (JSONArray) autoJSONs[i].getJSONArray("paths").getJSONObject(0).getJSONArray("sampled_points");
        autos[i] = new PolarAutoFollower(autoJSONs[i], drive, lights, peripherals, commandMap, conditionMap);
      } catch (Exception e) {
        System.out.println("ERROR LOADING PATH "+Constants.paths[i]+":" + e);
      }
    }

    lights.clearAnimations();
    lights.setCommandRunning(true);
    lights.setRGBFade();
    System.out.println("end robot init");
  }

  @Override
  public void robotPeriodic() {
    updateNoteInIntake();
    // System.out.println("is autonomous: " + DriverStation.isAutonomousEnabled());
    if (OI.getPOVUp()){
      // new DipShot(drive, shooter, feeder, peripherals, lights, proximity, 5, 5500, 1200, 0, 180, 180, 5).schedule();
      new PositionalFeederLobShot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 5).schedule();
    }
    // checks CAN and limelights, blinks green if good and blinks yellow if bad
    // System.out.println("checkedCan: " + checkedCAN);
    if (!checkedCAN) {
      if (Timer.getFPGATimestamp() - startTime > 30 || peripherals.limelightsConnected()) {
        checkedCAN = true;
        lights.clearAnimations();
        lights.setCommandRunning(false);
        if (drive.getSwerveCAN() && shooter.getShooterCAN() && intake.getIntakeCAN() && feeder.getFeederCAN()
            && climber.getClimberCAN() && peripherals.limelightsConnected()) {
          lights.blinkGreen(3);
        } else {
          lights.clearAnimations();
          lights.setCommandRunning(true);
          lights.setStrobeYellow();
        }
      }
    }

    shooterAngleDegreesTuning = SmartDashboard.getNumber("Shooter Angle Degrees (tuning)", 0);
    shooterRPMTuning = SmartDashboard.getNumber("Shooter RPM (input)", 0);
    CommandScheduler.getInstance().run();
    // System.out.println("Running");

    try {
      Logger.recordOutput("Localization Odometry", drive.getLocalizationOdometry());
    } catch (Exception e) {
      System.out.println("Problem with logging");
    }

    try {
      Logger.recordOutput("Wheel Odometry", drive.getOdometry());
    } catch (Exception e) {
      System.out.println("Problem with logging");
    }

    try {
      Logger.recordOutput("MT2 Odometry", drive.getMT2Odometry());
    } catch (Exception e) {
      System.out.println("Problem with logging");
    }
    Logger.recordOutput("Swerve Module States", drive.getModuleStates());
    Logger.recordOutput("Swerve Module Setpoints", drive.getModuleSetpoints());
    Logger.recordOutput("IMU", peripherals.getPigeonAngle());
    Logger.recordOutput("Note in Robot", getNoteInRobot());
    Constants.periodic();
    lights.periodic();
    intake.periodic();
    shooter.periodic();
    feeder.periodic();
    tof.periodic();
    proximity.periodic();

    // drive.periodic(); // remove for competition
    peripherals.periodic();
    climber.periodic();

    // System.out.println("0-1: " + (t1 - t0));

    // SmartDashboard.putNumber("Carriage Rotation",
    // climber.getCarriageRotationDegrees());
  }

  @Override
  public void disabledInit() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
    lights.clearAnimations();
    lights.setCommandRunning(true);
    lights.setRainbow();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    if (OI.isBlueSide()) {
      System.out.println("ON BLUE SIDE");
      fieldSide = "blue";
    } else {
      System.out.println("ON RED SIDE");
      fieldSide = "red";
    }
    this.drive.setFieldSide(fieldSide);
    this.peripherals.setFieldSide(fieldSide);

    System.out.print("Selected Auto: ");
    final int selectedPath = Constants.getSelectedPathIndex();
    if (selectedPath == -1) {
      System.out.println("Do Nothing");
      new DoNothing().schedule();
    } else {
      this.autos[selectedPath].schedule();
      this.drive.autoInit(autoPoints[selectedPath]);
      System.out.println("Selected Path: " + Constants.paths[selectedPath]);
    }
    this.intake.autoInit();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    shooter.teleopInit();
    lights.setCommandRunning(false);
    lights.clearAnimations();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (OI.isBlueSide()) {
      fieldSide = "blue";
    } else {
      fieldSide = "red";
    }

    if (this.fieldSide == "red") {
      this.drive.setPigeonAfterAuto();
    }
    System.out.println("field side" + fieldSide);

    this.peripherals.setFieldSide(fieldSide);
    this.drive.setFieldSide(fieldSide);

    // CONTROLS

    // Driver
    OI.driverViewButton.whileTrue(new ZeroAngleMidMatch(drive));
    OI.driverB.whileTrue(new PositionalLobShot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 5)); // tests CAN and Limelights, blinks green if good and blinks yellow if bad
    OI.driverRT.whileTrue(new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 450, true, true));
    OI.driverLT.whileTrue(new RunIntakeAndFeeder(intake, feeder, climber, Constants.SetPoints.IntakePosition.kUP, -800, -800, -0.4));
    // OI.operatorLB.whileTrue(new LobShot(drive, shooter, feeder, peripherals, lights, proximity, 55, 4400, 1200, 0, 193, 149, 5));
    OI.driverA.whileTrue(new AutoPositionalShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 22, 7000, false));
    OI.driverX.whileTrue(new DriveAutoAligned(drive, peripherals));
    OI.driverMenuButton
        .whileTrue(new PresetAutoShoot(drive, shooter, feeder, peripherals, lights, proximity, 60, 4500, 1200, 0, 1.5));
    OI.driverY.whileTrue(new DipShot(drive, shooter, feeder, peripherals, lights, proximity, 10, 6200, 1200, 0, 0, 0, 5));
    /*
     * auto align shot that is tunable, defaults to 0 degrees and 0 rpm but can be
     * changed in Smartdashboard
     */
    // OI.driverB.whileTrue(new PresetAutoShoot(drive, shooter, feeder, peripherals,
    // lights, proximity, shooterAngleDegreesTuning, shooterRPMTuning, 1200, 0, 2));

    // OI.driverRB.whileTrue(new MoveToPiece(drive, peripherals));

    // Operator
    // OI.operatorMenuButton.whileTrue(new PrepareAmp(climber, intake, feeder,
    // lights, peripherals, tof));
    OI.operatorX.whileTrue(new AmpPreset(climber, feeder, intake, proximity, shooter));
    OI.operatorB.whileTrue(new TrapPreset(climber, feeder, intake, proximity, shooter));
    OI.operatorY.whileTrue(new RunClimber(climber, feeder, 20, 1.0));
    OI.operatorA.whileTrue(new RunClimber(climber, feeder, -50, 1.0));
    OI.operatorRT.whileTrue(new AutoPrepForShot(shooter, proximity, 55, 4600));
    // OI.operatorRB.whileTrue(new SmartPrepForShot(shooter, peripherals, lights));
    OI.operatorRB.whileTrue(new PositionalSpinUp(drive, shooter, peripherals, lights, proximity));
    OI.operatorMenuButton.whileTrue(new RunFlywheel(shooter, 80, 0.2));
    OI.operatorViewButton
        .whileTrue(new AutoShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 22, 7000, false));
    // OI.operatorLB.whileTrue(new PositionalLobShot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 5));
    OI.operatorLT.whileTrue(new PositionalFeederLobShot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 5));
    OI.operatorLJ.whileTrue(new PositionalDipShot(drive, shooter, feeder, peripherals, lights, proximity, 5, 6200, 1200, 0, 5));
    // OI.operatorRB.whileTrue(new AutoIntake(intake, feeder, climber, lights, tof,
    // Constants.SetPoints.IntakePosition.kDOWN, 1200, 400));

  }

  @Override
  public void teleopPeriodic() {
    // SmartDashboard.putBoolean("Feeder TOF", this.tof.getFeederDistMillimeters() <
    // Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM);
    // SmartDashboard.putNumber("Feeder TOF Dist",
    // this.tof.getFeederDistMillimeters());
    intake.teleopPeriodic();
    shooter.teleopPeriodic();
    feeder.teleopPeriodic();
    climber.teleopPeriodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
