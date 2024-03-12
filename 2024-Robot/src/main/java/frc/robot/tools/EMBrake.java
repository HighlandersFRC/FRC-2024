package frc.robot.tools;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class EMBrake {
    Solenoid solenoid;
    public EMBrake(int portidx){
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, portidx);
    }
    public void lock(){
        solenoid.set(true);
    }
    public void unlock(){
        solenoid.set(false);
    }
    public void toggle(){
        if (isLocked()){
            unlock();
        } else {
            lock();
        }
    }
    public boolean isLocked(){
        return solenoid.get();
    }
}