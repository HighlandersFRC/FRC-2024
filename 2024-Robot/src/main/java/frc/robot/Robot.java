// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAutoAligned;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Lights lights = new Lights();
  private Peripherals peripherals = new Peripherals(lights);
  private Drive drive = new Drive(peripherals);
  
  File pathingFile;
  String pathString;

  JSONObject pathRead;
  JSONArray pathJSON;

  String fieldSide;

  SequentialCommandGroup auto;

  @Override
  public void robotInit() {
    if (OI.isRedSide()) {
      System.out.println("ON RED SIDE");
      fieldSide = "red";
    } else if (OI.isBlueSide()) {
      System.out.println("ON BLUE SIDE");
      fieldSide = "blue";
    }

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
