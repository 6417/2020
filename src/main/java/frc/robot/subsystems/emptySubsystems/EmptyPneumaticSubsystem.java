package frc.robot.subsystems.emptySubsystems;

import frc.robot.subsystems.PneumaticSubsystem;

public class EmptyPneumaticSubsystem extends PneumaticSubsystem {
    
    public EmptyPneumaticSubsystem() {
        super();
    }

    public void constructor() {

    }

    public boolean extendLift() {
        return true;
    }

    public void retractLift() {

    }

    public void closeLift() {

    }

    public void extendBumper() {

    }

    public boolean retractBumper() {
        return true;
    }

    public boolean closeBumper() {
        return true;
    }

    @Override
    public void periodic() {
        
    }
}