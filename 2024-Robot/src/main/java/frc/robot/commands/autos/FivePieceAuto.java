// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousFollower;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FivePieceAuto extends SequentialCommandGroup {
  private File pathingFile;
  private JSONArray pathJSON;
  private JSONObject pathRead;

  private File pathingFile2;
  private JSONArray pathJSON2;
  private JSONObject pathRead2;

  private File pathingFile3;
  private JSONArray pathJSON3;
  private JSONObject pathRead3;

  private File pathingFile4;
  private JSONArray pathJSON4;
  private JSONObject pathRead4;
  /** Creates a new FivePieceAuto. */
  public FivePieceAuto(Drive drive, Peripherals peripherals) {
    try {
      pathingFile = new File("/home/lvuser/deploy/2PieceCenterPart1.json");
      FileReader scanner = new FileReader(pathingFile);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile2 = new File("/home/lvuser/deploy/3PieceSpikePart2.json");
      FileReader scanner2 = new FileReader(pathingFile2);
      pathRead2 = new JSONObject(new JSONTokener(scanner2));
      pathJSON2 = (JSONArray) pathRead2.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile3 = new File("/home/lvuser/deploy/4PieceSpikePart3.json");
      FileReader scanner3 = new FileReader(pathingFile3);
      pathRead3 = new JSONObject(new JSONTokener(scanner3));
      pathJSON3 = (JSONArray) pathRead3.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    try {
      pathingFile4 = new File("/home/lvuser/deploy/4PieceClosePart4.json");
      FileReader scanner4 = new FileReader(pathingFile4);
      pathRead4 = new JSONObject(new JSONTokener(scanner4));
      pathJSON4 = (JSONArray) pathRead4.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }

    addRequirements(drive);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousFollower(drive, pathJSON, false),
      new AutonomousFollower(drive, pathJSON2, false),
      new AutonomousFollower(drive, pathJSON3, false),
      new AutonomousFollower(drive, pathJSON4, false)
    );
  }
}
