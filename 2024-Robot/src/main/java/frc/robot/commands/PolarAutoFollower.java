// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.JSONObject;

import java.util.HashMap;

import org.json.JSONArray;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PolarAutoFollower extends SequentialCommandGroup {
  /** Creates a new PolarAutoFollower. */
  public PolarAutoFollower(JSONObject polarAutoJSON, Drive drive, Lights lights, Peripherals peripherals, HashMap<String, Command> commandMap) {
    JSONArray schedule = polarAutoJSON.getJSONArray("schedule");
    JSONArray paths = polarAutoJSON.getJSONArray("paths");
    for (int i = 0; i < schedule.length(); i++) {
      JSONObject scheduleEntry = schedule.getJSONObject(i);
      if (!scheduleEntry.getBoolean("branched")){
        addCommands(
          new PolarPathFollower(drive, lights, peripherals, paths.getJSONObject(scheduleEntry.getInt("path")), commandMap)
        );
      }
    }
  }
}
