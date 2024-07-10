// Copyrights (c) 2018-2019 FIRST, 2020 Highlanders FRC. All Rights Reserved.
//hi om

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.tools.TriggerButton;

public class OI {
    public static XboxController driverController = new XboxController(0);
    public static XboxController operatorController = new XboxController(1);

    public static BooleanSupplier driveRTSupplier = () -> getDriverRTPercent() > Constants.OperatorConstants.RIGHT_TRIGGER_DEADZONE;
    public static BooleanSupplier driverLTSupplier = () -> getDriverLTPercent() > Constants.OperatorConstants.LEFT_TRIGGER_DEADZONE;

    public static TriggerButton driverRT = new TriggerButton(driveRTSupplier);
    public static TriggerButton driverLT = new TriggerButton(driverLTSupplier);

    public static JoystickButton driverA = new JoystickButton(driverController, 1);
    public static JoystickButton driverB = new JoystickButton(driverController, 2);

    public static JoystickButton driverY = new JoystickButton(driverController, 4);
    public static JoystickButton driverX = new JoystickButton(driverController, 3);

    public static JoystickButton driverRB = new JoystickButton(driverController, 6);
    public static JoystickButton driverLB = new JoystickButton(driverController, 5);

    public static JoystickButton operatorX = new JoystickButton(operatorController, 3);
    public static JoystickButton operatorB = new JoystickButton(operatorController, 2);

    public static JoystickButton operatorY = new JoystickButton(operatorController, 4);
    public static JoystickButton operatorA = new JoystickButton(operatorController, 1);

    

    public static BooleanSupplier operatorRTSupplier = () -> getOperatorRTPercent() > Constants.OperatorConstants.RIGHT_TRIGGER_DEADZONE;
    public static BooleanSupplier operatorLTSupplier = () -> getOperatorLTPercent() > Constants.OperatorConstants.LEFT_TRIGGER_DEADZONE;

    public static TriggerButton operatorRT = new TriggerButton(operatorRTSupplier);
    public static TriggerButton operatorLT = new TriggerButton(operatorLTSupplier);
    
    public static JoystickButton operatorRB = new JoystickButton(operatorController, 6);
    public static JoystickButton operatorLB = new JoystickButton(operatorController, 5);

    public static JoystickButton driverViewButton = new JoystickButton(driverController, 7);

    public static JoystickButton operatorViewButton = new JoystickButton(operatorController, 7);
    public static JoystickButton driverMenuButton = new JoystickButton(driverController, 8);

    public static JoystickButton operatorMenuButton = new JoystickButton(operatorController, 8);

    public static Joystick autoChooser = new Joystick(2);

    public static JoystickButton autoChooserIsBlue = new JoystickButton(autoChooser, 8);

    public static void printAutoChooserInputs(){
        System.out.println("Driver Controller Connected: " + driverController.isConnected());
        System.out.println("Operator Controller Connected: " + operatorController.isConnected());
        System.out.println("Auto Chooser Connected: " + autoChooser.isConnected());
        System.out.println("Auto Chooser Num Buttons: " + autoChooser.getButtonCount());
        System.out.println("Is Blue: " + autoChooserIsBlue.getAsBoolean());
        for (int i = 1; i <= 16;  i ++){
            System.out.println("Auto Chooser Button " + i + " : " + autoChooser.getRawButton(i));
        }
    }

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

    public static double getDriverRTPercent() {
        return driverController.getRightTriggerAxis();
    }

    public static double getDriverLTPercent() {
        return driverController.getLeftTriggerAxis();
    }

    public static boolean getDriverA() {
        return driverController.getAButton();
    }

    public static boolean getDriverRB(){
        return driverController.getRightBumper();
    }

    public static boolean getDriverLB(){
        return driverController.getLeftBumper();
    }

    public static double getOperatorRTPercent() {
        return operatorController.getRightTriggerAxis();
    }

    public static double getOperatorLTPercent() {
        return operatorController.getLeftTriggerAxis();
    }

    public static boolean getOperatorLB(){
        return operatorController.getLeftBumper();
    }

    public static int getPOV() {
        return driverController.getPOV();
    }

    public static boolean getPOVUp() {
        if (getPOV() == 0){
            return true;
        } else {
            return false;
        }
    }

    public static boolean isRedSide() {
        return autoChooser.getRawButton(6);
    }

    public static boolean isNothingAuto(){
        return autoChooser.getRawButton(7);
    }

    public static boolean autoChooserConnected() {
        return autoChooser.isConnected();
    }

    public static boolean isBlueSide() {
        return autoChooser.getRawButton(8);
    }

    public static boolean is4PieceFarBottom231Auto(){
        return autoChooser.getRawButton(2);
    }

    public static boolean is5PieceAuto() {
        return autoChooser.getRawButton(1);
    }

    public static boolean is3PieceFarBottomAuto() {
        return autoChooser.getRawButton(3);
    }

    public static boolean is1PieceAuto() {
        return autoChooser.getRawButton(5);
    }

    public static boolean is4PieceAmpSideAuto() {
        return autoChooser.getRawButton(4);
    }

    public static boolean getDriverRightJoystickPressed(){
        return driverController.getRightStickButton();
    }
}