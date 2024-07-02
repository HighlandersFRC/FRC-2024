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
import frc.robot.sensors.TOF;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoNoteFollowing extends SequentialCommandGroup {
  private File pathingFile;
  private JSONArray pathJSON;
  private JSONObject pathRead;
  /** Creates a new AutoNoteFollowing. */
  public AutoNoteFollowing(Drive drive, Peripherals peripherals, Intake intake, Feeder feeder, Climber climber, Lights lights, TOF tof) {
    try {
      pathingFile = new File("/home/lvuser/deploy/AutoNoteFollowingTest.json");
      FileReader scanner = new FileReader(pathingFile);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }
    
    addRequirements(drive, peripherals);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ParallelDeadlineGroup(
        // new AutonomousFollower(drive, lights, peripherals, pathJSON, 0, false, true, 0)
      // )
    );
  }
}
