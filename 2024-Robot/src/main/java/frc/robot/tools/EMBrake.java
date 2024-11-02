package frc.robot.tools;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class EMBrake {
    Solenoid m_solenoid;

    public EMBrake(int portidx) {
        m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, portidx);
    }

    /**
     * Locks the embrake by setting the solenoid output to true.
     * This will prevent the robot from moving forward or backward.
     *
     * @return void
     */
    public void lock() {
        m_solenoid.set(true);
    }

    /**
     * Unlocks the embrake by setting the solenoid output to false.
     * This will allow the robot to move forward or backward.
     *
     * @return void
     */
    public void unlock() {
        m_solenoid.set(false);
    }

    /**
     * Toggles the embrake state between locked and unlocked.
     * If the embrake is currently locked, it will be unlocked, and vice versa.
     *
     * @return void
     */
    public void toggle() {
        if (isLocked()) {
            unlock();
        } else {
            lock();
        }
    }

    /**
     * Checks if the embrake is currently locked.
     * The embrake is considered locked when the solenoid output is true,
     * preventing the robot from moving forward or backward.
     *
     * @return true if the embrake is locked, false otherwise
     */
    public boolean isLocked() {
        return m_solenoid.get();
    }
}