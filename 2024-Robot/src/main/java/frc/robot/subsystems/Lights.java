// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
// import frc.robot.sensors.TOF;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private String fieldSide = "none";
  private boolean commandRunning = false;
  private double time = Timer.getFPGATimestamp();
  private double timeout = 0.0;
  private boolean timedFlashes = false;
  CANdle candle = new CANdle(Constants.CANInfo.CANDLE_ID, Constants.CANInfo.CANBUS_NAME);
  
  RgbFadeAnimation rgbFade = new RgbFadeAnimation(1, 0.2, 308);
  RainbowAnimation rainbowAnimation = new RainbowAnimation(1, 0.4, 308, true, 0);
  StrobeAnimation flashGreen = new StrobeAnimation(0, 255, 0, 0, 0.7, 308, 0);
  StrobeAnimation flashPurple = new StrobeAnimation(255, 0, 255, 0, 0.7, 308, 0);
  StrobeAnimation flashYellow = new StrobeAnimation(255, 255, 0, 0, 0.5, 308, 0);
  // private TOF tof;

  /*
   * Lights codes are as follows:
   * solid yellow - robot can't see auto chooser (might mean that robot is disconected)
   * flashing yellow - error: usually means that the robot cannot see all of its CAN devices and limelights
   * solid red - red alliance
   * solid blue - blue alliance
   * flashing purple:
   *      autonomous - the robot does not see the note
   *      intaking - robot has not intaken note yet
   *      shooting - robot cannot see apriltag
   * flashing green:
   *      autonomous - the robot sees the note
   *      boot up/CAN check - all CAN is good and limelights are connected
   *      intake - robot has intaken note
   *      shooting - robot can see apriltag
   * solid green:
   *      shooting - robot is aligned and ready to shoot 
   */

  /**
   * Constructs a new Lights object.
   * 
   * @param tof Time of Flight sensor.
  */
  public Lights() {
    // this.tof = tof;
  }

  public void setCommandRunning(boolean commandRunning) { // used to bypass the default light colors (red/blue)
    this.commandRunning = commandRunning;
  }

  /**
   * Sets the RGB values of the lights to the specified values.
   *
   * @param r The red component value (0-255).
   * @param g The green component value (0-255).
   * @param b The blue component value (0-255).
  */
  public void setCandleRGB(int r, int g, int b) { // sets the RGB values of the lights
    candle.setLEDs(r, g, b);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!commandRunning) { // only makes lights red/blue if a command is not trying to change light colors
      if(!OI.autoChooserConnected()) {
        fieldSide = "none";
      } else if (OI.isBlueSide()) {
        fieldSide = "blue";
      } else {
        fieldSide = "red";
      }

      if (fieldSide == "red") { // sets lights to color of alliance
        candle.setLEDs(255, 0, 0);
      } else if (fieldSide == "blue") {
        candle.setLEDs(0, 0, 255);
      } else if(fieldSide == "none") {
        candle.setLEDs(255, 255, 0);
      } else {
        candle.setLEDs(255, 255, 255);
      }
    } else if (timedFlashes) { // allows lights to flash for a certain period of time before returning to default colors
      if(Timer.getFPGATimestamp() - time > timeout) {
        timedFlashes = false;
        candle.clearAnimation(0);
        setCommandRunning(false);
      }
    }
  }
  /**
   * Sets the flag indicating whether timed flashes are enabled or disabled.
   * 
   * @param timedFlashes a boolean value indicating whether timed flashes should be enabled (true) or disabled (false).
  */
  public void setTimedFlashes(boolean timedFlashes) {
    this.timedFlashes = timedFlashes;
  }

  public void blinkGreen(double seconds) { // blinks green for a certain amount of time
    setCommandRunning(true);
    candle.clearAnimation(0);
    if(seconds != -1) {
      time = Timer.getFPGATimestamp();
      timeout = seconds;
      timedFlashes = true;
    }
    candle.animate(flashGreen);
  }

  public void blinkYellow(double seconds) { // blinks yellow for a certain amount of time
    setCommandRunning(true);
    candle.clearAnimation(0);
    if(seconds != -1) {
      time = Timer.getFPGATimestamp();
      timeout = seconds;
      timedFlashes = true;
    }
    candle.animate(flashYellow);
  }

  public void clearAnimations() { // clears all animations currently running
    candle.clearAnimation(0);
  }

  public void setStrobeGreen() { // flashing green animation
    candle.animate(flashGreen);
  }

  public void setRainbow() {
    candle.animate(rainbowAnimation);
  }

  public void setStrobePurple() { // flashing purple animation
    candle.animate(flashPurple);
  }

  /**
   * Sets the candle object to display a flashing yellow animation.
   * The animation is defined by the flashYellow object.
  */
  public void setStrobeYellow() { // flashing yellow animation
    candle.animate(flashYellow);
  }

  public void setRGBFade() {
    candle.animate(rgbFade);
  }

  /**
   * Initializes the Lights subsystem with the specified field side.
   * Clears animation at index 0 of the candle object.
   * 
   * @param fieldSide The side of the field to initialize with.
  */
  public void init(String fieldSide){
    this.fieldSide = fieldSide;
    candle.clearAnimation(0);
  }
}
