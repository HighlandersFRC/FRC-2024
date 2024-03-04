// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.TOF;
import frc.robot.commands.AutoShoot;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private String fieldSide = "red";
  CANdle candle = new CANdle(Constants.CANInfo.CANDLE_ID, Constants.CANInfo.CANBUS_NAME);
  RainbowAnimation rainbowAnimation = new RainbowAnimation(1, 0.4, 308, true, 0);
  StrobeAnimation flashGreen = new StrobeAnimation(0, 255, 0, 0, 0.7, 308, 0);
  StrobeAnimation flashPurple = new StrobeAnimation(127, 0, 255, 0, 0.7, 308, 0);
  private TOF tof;

  public Lights() {
  }

  public void Candle(){
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    candle.animate(rainbowAnimation);
    if(TOF.intakeTOF.getRange() <= Constants.SetPoints.INTAKE_TOF_THRESHOLD_MM){
      candle.animate(flashGreen);
    } else {
      candle.animate(rainbowAnimation);
    }

    if(!AutoShoot.canSeeTag){
      candle.setLEDs(255, 0 ,0);
    } else if (AutoShoot.canSeeTag) {
      candle.setLEDs(0, 255, 0);
    } else {
      candle.animate(rainbowAnimation);
    }
    
  }

  public void init(String fieldSide){
    this.fieldSide = fieldSide;
  }
}
