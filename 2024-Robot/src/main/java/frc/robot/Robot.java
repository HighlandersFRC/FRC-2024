// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileReader;

import javax.sound.sampled.SourceDataLine;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoParser;
import frc.robot.commands.AutoPrepForShot;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.DriveAutoAligned;
import frc.robot.commands.IndexNoteToCarriage;
import frc.robot.commands.LobShot;
import frc.robot.commands.MoveOnlyArm;
import frc.robot.commands.MoveToPiece;
// import frc.robot.commands.PrepareAmp;
import frc.robot.commands.PresetAutoShoot;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunFlywheel;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeAndFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTrap;
import frc.robot.commands.SetClimber;
import frc.robot.commands.SmartIntake;
import frc.robot.commands.SmartPrepForShot;
import frc.robot.commands.SmartShoot;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.Test;
import frc.robot.commands.TestCAN;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.commands.autos.AutoNoteFollowing;
import frc.robot.commands.autos.FivePieceAuto;
import frc.robot.commands.autos.FourPieceCloseAuto;
import frc.robot.commands.autos.FourPieceFarBottomAuto;
import frc.robot.commands.autos.NothingAuto;
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

  //Sensors
  private TOF tof = new TOF();
  private Proximity proximity = new Proximity();

  //Subsystems
  private Lights lights = new Lights(tof);
  private Peripherals peripherals = new Peripherals();
  private Drive drive = new Drive(peripherals);
  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private Feeder feeder = new Feeder(tof, proximity);
  private Climber climber = new Climber(lights, tof, proximity);

  // private Logger logger = Logger.getInstance();

  private double shooterAngleDegreesTuning = 0;
  private double shooterRPMTuning = 0;
  private double startTime = Timer.getFPGATimestamp();
  private boolean checkedCAN = false;

  Command nothingAuto;
  
  File fourPieceCloseFile;
  JSONArray fourPieceCloseJSON;
  Command fourPieceCloseAuto;

  File fourPieceFarBottomFile;
  JSONArray fourPieceFarBottomJSON;
  Command fourPieceFarBottomAuto;

  File fivePieceFile;
  JSONArray fivePieceJSON;
  Command fivePieceAuto;

  File autoNoteFollowingFile;
  JSONArray autoNoteFollowingJSON;
  Command autoNoteFollowingAuto;

  String fieldSide = "blue";

  @Override
  public void robotInit() {
    // System.out.println("Starting");
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    // Logger.recordMetadata("Code", "Running");
    // Logger.disableDeterministicTimestamps(); // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
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
    try {
      this.fourPieceCloseFile = new File("/home/lvuser/deploy/4PieceClosePart1.json");
      FileReader scanner = new FileReader(this.fourPieceCloseFile);
      JSONObject pathRead = new JSONObject(new JSONTokener(scanner));
      this.fourPieceCloseJSON = (JSONArray) pathRead.get("sampled_points");
      this.fourPieceCloseAuto = new FourPieceCloseAuto(drive, peripherals, intake, feeder, shooter, climber, lights, tof, proximity);
    } catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      this.fourPieceFarBottomFile = new File("/home/lvuser/deploy/4PieceFarBottomPart1.json");
      FileReader scanner = new FileReader(this.fourPieceFarBottomFile);
      JSONObject pathRead = new JSONObject(new JSONTokener(scanner));
      this.fourPieceFarBottomJSON = (JSONArray) pathRead.get("sampled_points");
      this.fourPieceFarBottomAuto = new FourPieceFarBottomAuto(drive, peripherals, intake, feeder, shooter, climber, lights, tof, proximity);
    } catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      this.fivePieceFile = new File("/home/lvuser/deploy/4PieceClosePart1.json");
      FileReader scanner = new FileReader(this.fivePieceFile);
      JSONObject pathRead = new JSONObject(new JSONTokener(scanner));
      this.fivePieceJSON = (JSONArray) pathRead.get("sampled_points");
      this.fivePieceAuto = new FivePieceAuto(drive, peripherals, intake, feeder, shooter, climber, lights, tof, proximity);
    } catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      this.autoNoteFollowingFile = new File("/home/lvuser/deploy/AutoNoteFollowingTest.json");
      FileReader scanner = new FileReader(this.autoNoteFollowingFile);
      JSONObject pathRead = new JSONObject(new JSONTokener(scanner));
      this.autoNoteFollowingJSON = (JSONArray) pathRead.get("sampled_points");
      this.autoNoteFollowingAuto = new AutoNoteFollowing(drive, peripherals, intake, feeder, climber, lights, tof);
    } catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    lights.clearAnimations();
    lights.setCommandRunning(true);
    lights.setRGBFade();

  }
 
  @Override
  public void robotPeriodic() {

        // checks CAN and limelights, blinks green if good and blinks yellow if bad
    // System.out.println("checkedCan: " + checkedCAN);
    if (!checkedCAN){
      if(Timer.getFPGATimestamp() - startTime > 30 || peripherals.limelightsConnected()) {
        checkedCAN = true;
        lights.clearAnimations();
        lights.setCommandRunning(false);
        if(drive.getSwerveCAN() && shooter.getShooterCAN() && intake.getIntakeCAN() && feeder.getFeederCAN() && climber.getClimberCAN() && peripherals.limelightsConnected()) {
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

    try{
      Logger.recordOutput("Localization Odometry", drive.getLocalizationOdometry());
    } catch(Exception e) {
      System.out.println("Problem with logging");
    }
    // Logger.recordOutput("Swerve Module States", drive.getModuleStates());
    // Logger.recordOutput("Swerve Module Setpoints", drive.getModuleSetpoints());

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

    // SmartDashboard.putNumber("Carriage Rotation", climber.getCarriageRotationDegrees());
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
  public void disabledPeriodic() {}

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

    System.out.println("Selected Auto: ");
    if (OI.isNothingAuto()){
      System.out.println("Nothing Auto");
      this.nothingAuto.schedule();
    } else if (OI.is4PieceCloseAuto()) {
      System.out.println("Four Piece Close");
      this.fourPieceCloseAuto.schedule();
      this.drive.autoInit(this.fourPieceCloseJSON);
    } else if (OI.is3PieceFarBottomAuto()){
      System.out.println("Four Piece Far Bottom");
      this.fourPieceFarBottomAuto.schedule();
      this.drive.autoInit(this.fourPieceFarBottomJSON);
    } else if (OI.is5PieceAuto()){
      System.out.println("Five Piece");
      this.fivePieceAuto.schedule();
      this.drive.autoInit(this.fivePieceJSON);
    }else {
      System.out.println("NO AUTO SELECTED");
    }

    this.intake.autoInit();
  }

  @Override
  public void autonomousPeriodic() {}

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

    if (this.fieldSide == "red"){
      this.drive.setPigeonAfterAuto();
    }

    this.peripherals.setFieldSide(fieldSide);

    //CONTROLS

    //Driver
    OI.driverViewButton.whileTrue(new ZeroAngleMidMatch(drive));
    OI.driverMenuButton.whileTrue(new MoveOnlyArm(climber, 190)); // tests CAN and Limelights, blinks green if good and blinks yellow if bad
    OI.driverRT.whileTrue(new AutoIntake(intake, feeder, climber, lights, tof, proximity, Constants.SetPoints.IntakePosition.kDOWN, 1200, 450, true, true));
    OI.driverLT.whileTrue(new RunIntakeAndFeeder(intake, feeder, climber, Constants.SetPoints.IntakePosition.kUP, -800, -800, -0.4));
    OI.driverY.whileTrue(new LobShot(drive, shooter, feeder, peripherals, lights, proximity, 45, 4100, 1200, 0, 5));
    OI.driverA.whileTrue(new AutoShoot(drive, shooter, feeder, peripherals, lights, proximity, 1200, 22, 7000, 3));
    OI.driverX.whileTrue(new PresetAutoShoot(drive, shooter, feeder, peripherals, lights, proximity, 64, 4500, 1200, 0, 1.5));
    OI.driverB.whileTrue(new PresetAutoShoot(drive, shooter, feeder, peripherals, lights, proximity, 24.5, 7000, 1200, 0, 2.5));

    /* auto align shot that is tunable, defaults to 0 degrees and 0 rpm but can be changed in Smartdashboard */
    // OI.driverB.whileTrue(new PresetAutoShoot(drive, shooter, feeder, peripherals, lights, proximity, shooterAngleDegreesTuning, shooterRPMTuning, 1200, 0, 2));
    
    // OI.driverRB.whileTrue(new MoveToPiece(drive, peripherals));

    //Operator
    // OI.operatorMenuButton.whileTrue(new PrepareAmp(climber, intake, feeder, lights, peripherals, tof));
    OI.operatorX.whileTrue(new AmpPreset(climber, feeder, intake, proximity, shooter));
    OI.operatorB.whileTrue(new TrapPreset(climber, feeder, intake, proximity, shooter));
    OI.operatorY.whileTrue(new RunClimber(climber, 20, 1.0));
    OI.operatorA.whileTrue(new RunClimber(climber, -20, 1.0));

    OI.operatorRB.whileTrue(new SmartPrepForShot(shooter, peripherals, lights));
    OI.operatorMenuButton.whileTrue(new RunFlywheel(shooter, 80, 0.2));

    // OI.operatorRB.whileTrue(new AutoIntake(intake, feeder, climber, lights, tof, Constants.SetPoints.IntakePosition.kDOWN, 1200, 400));
    
  }

  @Override
  public void teleopPeriodic() {
    // SmartDashboard.putBoolean("Feeder TOF", this.tof.getFeederDistMillimeters() < Constants.SetPoints.FEEDER_TOF_THRESHOLD_MM);
    // SmartDashboard.putNumber("Feeder TOF Dist", this.tof.getFeederDistMillimeters());
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
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
