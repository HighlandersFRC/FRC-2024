package frc.robot.tools;


import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerButton extends Trigger {
	public TriggerButton(BooleanSupplier bs) {
		super(bs);
	}
}