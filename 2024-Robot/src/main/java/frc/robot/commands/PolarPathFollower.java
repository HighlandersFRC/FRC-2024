// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PolarPathFollower extends ParallelCommandGroup {
  /** Creates a new PolarPathFollower. */
  PurePursuitFollower follower;
  double endTime = 0;
  boolean timerStarted = false;
  JSONObject pathJSON;

  public PolarPathFollower(Drive drive, Lights lights, Peripherals peripherals, JSONObject pathJSON,
      HashMap<String, Command> commandMap) {
    follower = new PurePursuitFollower(drive, lights, peripherals, pathJSON.getJSONArray("sampled_points"), false);
    this.pathJSON = pathJSON;
    ArrayList<Command> commands = new ArrayList<>();
    commands.add(follower);
    for (int i = 0; i < pathJSON.getJSONArray("commands").length(); i++) {
      JSONObject command = pathJSON.getJSONArray("commands").getJSONObject(i);
      if (command.has("command")) {
        BooleanSupplier startSupplier = () -> command.getDouble("start") < getPathTime();
        BooleanSupplier endSupplier = () -> command.getDouble("end") <= getPathTime();
        commands.add(
            new TriggerCommand(startSupplier, commandMap.get(command.getJSONObject("command").getString("name")),
                endSupplier));
      }
    }
    for (Command command : commands) {
      addCommands(command);
    }
  }

  double getPathTime() {
    double retval;
    if (follower.isFinished()) {
      if (!timerStarted) {
        endTime = Timer.getFPGATimestamp();
        timerStarted = true;
      }
      retval = Timer.getFPGATimestamp() - endTime + pathJSON.getJSONArray("sampled_points")
          .getJSONObject(pathJSON.getJSONArray("sampled_points").length() - 1).getDouble("time");
    } else {
      retval = pathJSON.getJSONArray("sampled_points")
          .getJSONObject(follower.getPathPointIndex()).getDouble("time");
    }
    Logger.recordOutput("Path Time", retval);
    return retval;
  }
}
