// Copyrights (c) 2018-2019 FIRST, 2020 Highlanders FRC. All Rights Reserved.
//hi om

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.tools.TriggerButton;

public class OI {
    public static XboxController driverController = new XboxController(0);
    public static XboxController operatorController = new XboxController(1);

    public static JoystickButton driverA = new JoystickButton(driverController, 1);
    public static JoystickButton driverB = new JoystickButton(driverController, 2);

    public static JoystickButton driverY = new JoystickButton(driverController, 4);
    public static JoystickButton driverX = new JoystickButton(driverController, 3);

    // public static TriggerButton driverRT = new TriggerButton(driverController, 3);
    // public static TriggerButton driverLT = new TriggerButton(driverController, 2);
    public static JoystickButton driverRB = new JoystickButton(driverController, 6);
    public static JoystickButton driverLB = new JoystickButton(driverController, 5);

    public static JoystickButton operatorX = new JoystickButton(operatorController, 3);
    public static JoystickButton operatorB = new JoystickButton(operatorController, 2);

    public static JoystickButton operatorY = new JoystickButton(operatorController, 4);
    public static JoystickButton operatorA = new JoystickButton(operatorController, 1);
    
    // public static TriggerButton operatorRT = new TriggerButton(operatorController, 3);
    // public static TriggerButton operatorLT = new TriggerButton(operatorController, 2);
    public static JoystickButton operatorRB = new JoystickButton(operatorController, 6);
    public static JoystickButton operatorLB = new JoystickButton(operatorController, 5);

    public static JoystickButton driverViewButton = new JoystickButton(driverController, 7);

    public static JoystickButton operatorViewButton = new JoystickButton(operatorController, 7);
    public static JoystickButton driverMenuButton = new JoystickButton(driverController, 8);

    public static JoystickButton operatorMenuButton = new JoystickButton(operatorController, 8);

    public static Joystick autoChooser = new Joystick(2);

    public static double getDriverLeftX() {
        return -driverController.getLeftX();
    }

    public static double getDriverLeftY() {
        return driverController.getLeftY();
    }

    public static double getDriverRightX() {
        return driverController.getRightX();
    }

    public static double getDriverRightY() {
        return driverController.getRightY();
    }

    public static boolean getDriverA() {
        return driverController.getAButton();
    }

    public static int getPOV() {
        return driverController.getPOV();
    }

    public static boolean isRedSide() {
        return autoChooser.getRawButton(6);
    }

    public static boolean isBlueSide() {
        return autoChooser.getRawButton(8);
    }

    public static boolean is4PieceCloseAuto(){
        return autoChooser.getRawButton(1);
    }

    public static boolean is5PieceAuto() {
        return autoChooser.getRawButton(2);
    }

    public static boolean is4Piece1FarAuto() {
        return autoChooser.getRawButton(3);
    }

    public static boolean is3PieceBottomAuto() {
        return autoChooser.getRawButton(4);
    }

    public static boolean is4Piece2FarAuto(){
        return autoChooser.getRawButton(5);
    }

    public static boolean isPodiumPickup() {
        return autoChooser.getRawButton(7);
    }
}