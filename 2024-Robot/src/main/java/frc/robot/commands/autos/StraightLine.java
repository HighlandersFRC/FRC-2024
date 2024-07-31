// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PurePursuitFollower;
import frc.robot.sensors.Proximity;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightLine extends SequentialCommandGroup {
  /** Creates a new StraightLine. */
  File pathingFile;
  JSONObject pathRead;
  JSONArray pathJSON;
  public StraightLine(Drive drive, Lights lights, Peripherals peripherals) {
    try {
      pathingFile = new File(Filesystem.getDeployDirectory().getPath() + "/Square.polarpath");
      FileReader scanner = new FileReader(pathingFile);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }
    addRequirements(drive);
    addCommands(
      new PurePursuitFollower(drive, lights, peripherals, pathJSON, false)
    );
  }
}
