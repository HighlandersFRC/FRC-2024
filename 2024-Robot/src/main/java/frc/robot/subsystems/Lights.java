// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.sensors.TOF;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.defaults.DriveDefault;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private String fieldSide = "none";
  private boolean commandRunning = false;
  private Boolean flip = true;
  CANdle candle = new CANdle(Constants.CANInfo.CANDLE_ID, Constants.CANInfo.CANBUS_NAME);
  
  RainbowAnimation rainbowAnimation = new RainbowAnimation(1, 0.4, 308, true, 0);
  StrobeAnimation flashGreen = new StrobeAnimation(0, 255, 0, 0, 0.7, 308, 0);
  StrobeAnimation flashPurple = new StrobeAnimation(255, 0, 255, 0, 0.7, 308, 0);
  StrobeAnimation flashYellow = new StrobeAnimation(255, 255, 0, 0, 0.7, 308, 0);
  private TOF tof;

  public Lights(TOF tof) {
    this.tof = tof;
  }

  public void Candle(){
    
  }

  public void setCommandRunning(boolean input) {
    commandRunning = input;
  }

  public void setCandleRGB(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // candle.animate(rainbowAnimation);
    if(!commandRunning) {
      if(OI.autoChooserIsBlue.getAsBoolean() && OI.autoChooserIsRed.getAsBoolean()) {
        fieldSide = "both";
      } else if (OI.autoChooserIsBlue.getAsBoolean()) {
        fieldSide = "blue";
      } else if (OI.autoChooserIsRed.getAsBoolean()) {
        fieldSide = "red";
      } else {
        fieldSide = "none";
      }
      SmartDashboard.putString("Field Side", fieldSide);
      if (fieldSide == "red") {
        candle.setLEDs(255, 0, 0);
      } else if (fieldSide == "blue") {
        candle.setLEDs(0, 0, 255);
      } else if(fieldSide == "none") {
        candle.setLEDs(255, 255, 0);
      } else if(fieldSide == "both") {
        if (flip) {
          flip = false;
          candle.setLEDs(255, 0, 0);
        } else {
          flip = true;
          candle.setLEDs(0, 0, 255);
        }
      } else {
        candle.setLEDs(255, 255, 255);
      }
    }
  }

  public void clearAnimations() {
    candle.clearAnimation(0);
  }

  public void setStrobeGreen() {
    candle.animate(flashGreen);
  }

  public void setStrobePurple() {
    candle.animate(flashPurple);
  }

  public void init(String fieldSide){
    this.fieldSide = fieldSide;
    candle.clearAnimation(0);
  }
}
