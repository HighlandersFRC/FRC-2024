// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAutoAligned;
import frc.robot.commands.Shoot;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Lights lights = new Lights();
  private Peripherals peripherals = new Peripherals(lights);
  private Drive drive = new Drive(peripherals);
  private Shooter shooter = new Shooter();
  
  File pathingFile;
  String pathString;

  JSONObject pathRead;
  JSONArray pathJSON;

  String fieldSide;

  SequentialCommandGroup auto;

  @Override
  public void robotInit() {
    lights.init(fieldSide);
    peripherals.init();
    drive.init(fieldSide);

    PortForwarder.add(5800, "limelight-front.local", 5800);
    PortForwarder.add(5801, "limelight-front.local", 5801);

    PortForwarder.add(5800, "limelight-back.local", 5800);
    PortForwarder.add(5801, "limelight-back.local", 5801);

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);

    PortForwarder.add(5800, "10.44.99.39", 5800);
    PortForwarder.add(5801, "10.44.99.39", 5801);

    PortForwarder.add(5800, "10.44.99.40", 5800);
    PortForwarder.add(5801, "10.44.99.40", 5801);

    //Auto selection here...
    if (OI.isRedSide()) {
      System.out.println("ON RED SIDE");
      fieldSide = "red";
    } else if (OI.isBlueSide()) {
      System.out.println("ON BLUE SIDE");
      fieldSide = "blue";
    }

    // if (OI.is4PieceCloseAuto()) {
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/2PieceCenterPart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch(Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // } else if (OI.is5PieceAuto()) {
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/2PieceCenterPart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // } else if (OI.is4Piece1FarAuto()) {
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/2PieceBumpPart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // } else if (OI.is3PieceBottomAuto()) {
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/2PiecePart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // } else if (OI.is4Piece2FarAuto()){
    //   try {
    //     pathingFile = new File("/home/lvuser/deploy/2PiecePart1.json");
    //     FileReader scanner = new FileReader(pathingFile);
    //     pathRead = new JSONObject(new JSONTokener(scanner));
    //     pathJSON = (JSONArray) pathRead.get("sampled_points");
    //   } catch (Exception e) {
    //     System.out.println("ERROR WITH PATH FILE " + e);
    //   }
    // }

    // if (OI.is2PieceBumpSideAuto()) {
    //   drive.useCameraInOdometry();
    //   if (OI.isDocking()) {
    //     this.auto = new TwoPieceBumpAuto(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
    //     auto.schedule();
    //   } else {
    //     this.auto = new TwoPieceBumpAutoNoDock(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
    //     auto.schedule();
    //   }
    // } else if (OI.is1PieceAuto()) {
    //   if (OI.isDocking()) {
    //     this.auto = new OnePieceAuto(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
    //     auto.schedule();
    //   } else {
    //     this.auto = new OnePieceAutoNoDock(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
    //     auto.schedule();
    //   }
    // // } else if (OI.is2Plus1BumpSideAuto()) {
    // //   if (OI.isDocking()){
    // //     this.auto = new TwoPlusOneBumpAuto(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
    // //     auto.schedule();
    // //   } else {
    // //     this.auto = new TwoPlusOneBumpAutoNoDock(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
    // //     auto.schedule();
    // //   }
    // } else if (OI.is3PieceBumpSideAuto()){
    //   this.auto = new ThreePieceBumpAuto(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
    //   auto.schedule();
    // } else if (OI.is2Plus1ClearSideAuto()) {
    //   if (OI.isDocking()){
    //     this.auto = new TwoPlusOneAuto(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
    //     auto.schedule();
    //   } else {
    //     this.auto = new TwoPlusOneAutoNoDock(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
    //     auto.schedule();
    //   }
    // } else if (OI.is3PieceClearSideAuto()){
    //   this.auto = new ThreePieceAuto(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
    //   auto.schedule();
    // } else {
    //   System.out.println("NO AUTO SELECTED");
    // }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    lights.periodic();
  }

  @Override
  public void disabledInit() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    try {
      this.auto.schedule();
    } catch (Exception e){
      System.out.println("No auto is selected");
    } 
    drive.autoInit(this.pathJSON);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //CONTROLS
    OI.driverX.whileTrue(new DriveAutoAligned(drive, peripherals));
    OI.driverViewButton.whileTrue(new ZeroAngleMidMatch(drive));
    OI.driverA.whileTrue(new Shoot(shooter, 0.6));
  }

  @Override
  public void teleopPeriodic() {}

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
