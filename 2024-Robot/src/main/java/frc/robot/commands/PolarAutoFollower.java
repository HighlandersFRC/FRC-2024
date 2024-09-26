// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.JSONObject;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PolarAutoFollower extends SequentialCommandGroup {
  /** Creates a new PolarAutoFollower. */
  public PolarAutoFollower(JSONObject polarAutoJSON, Drive drive, Lights lights, Peripherals peripherals,
      HashMap<String, Supplier<Command>> commandMap, HashMap<String, BooleanSupplier> conditionMap) {
    JSONArray schedule = polarAutoJSON.getJSONArray("schedule");
    JSONArray paths = polarAutoJSON.getJSONArray("paths");
    for (int i = 0; i < schedule.length(); i++) {
      JSONObject scheduleEntry = schedule.getJSONObject(i);
      if (!scheduleEntry.getBoolean("branched")) {
        addCommands(
            new PolarPathFollower(drive, lights, peripherals, paths.getJSONObject(scheduleEntry.getInt("path")),
                commandMap, conditionMap));
      } else {
        JSONArray onTrueSchedule = scheduleEntry.getJSONObject("branched_path").getJSONArray("on_true");
        JSONArray onFalseSchedule = scheduleEntry.getJSONObject("branched_path").getJSONArray("on_false");
        JSONObject onTrueJSON = new JSONObject();
        JSONObject onFalseJSON = new JSONObject();
        onTrueJSON.append("paths", new JSONArray());
        onFalseJSON.append("paths", new JSONArray());
        for (int j = 0; j < paths.length(); j++) {
          JSONObject path = paths.getJSONObject(j);
          onTrueJSON.getJSONArray("paths").put(path);
          onFalseJSON.getJSONArray("paths").put(path);
        }
        onTrueJSON.getJSONArray("paths").remove(0);
        onFalseJSON.getJSONArray("paths").remove(0);
        onTrueJSON.append("schedule", new JSONArray());
        onFalseJSON.append("schedule", new JSONArray());
        for (int j = 0; j < onTrueSchedule.length(); j++) {
          JSONObject sched = onTrueSchedule.getJSONObject(j);
          onTrueJSON.getJSONArray("schedule").put(sched);
        }
        for (int j = 0; j < onFalseSchedule.length(); j++) {
          JSONObject sched = onFalseSchedule.getJSONObject(j);
          onFalseJSON.getJSONArray("schedule").put(sched);
        }
        onTrueJSON.getJSONArray("schedule").remove(0);
        onFalseJSON.getJSONArray("schedule").remove(0);
        BooleanSupplier condition = conditionMap.get(scheduleEntry.get("condition"));
        Runnable onTrueRunnable = new Runnable() {
          public void run() {
            System.out.println("True Path Starting");
            System.out.println(condition.getAsBoolean());
          }
        };
        Runnable onFalseRunnable = new Runnable() {
          public void run() {
            System.out.println("False Path Starting");
            System.out.println(condition.getAsBoolean());
          }
        };
        addCommands(
            new ConditionalCommand(
                new PolarAutoFollower(onTrueJSON, drive, lights, peripherals, commandMap, conditionMap)
                    .beforeStarting(onTrueRunnable),
                new PolarAutoFollower(onFalseJSON, drive, lights, peripherals, commandMap, conditionMap)
                    .beforeStarting(onFalseRunnable),
                condition));
      }
    }
  }
}
