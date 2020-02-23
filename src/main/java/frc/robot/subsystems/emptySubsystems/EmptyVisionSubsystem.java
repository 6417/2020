package frc.robot.subsystems.emptySubsystems;

import frc.robot.subsystems.VisionSubsystem;

public class EmptyVisionSubsystem extends VisionSubsystem {
    public EmptyVisionSubsystem() {

    }

    protected void constructor() {
        
    }

    @Override
    public void periodic() {
        
    }

    public double getDistance() {
        return 0;
    }

    public double getAngle() {
        return 0;
    }

    public boolean targetDetected() {
        return false;
    }

    public double getOffset() {
        return 0;
    }
}