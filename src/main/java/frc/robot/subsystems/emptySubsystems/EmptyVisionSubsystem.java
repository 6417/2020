package frc.robot.subsystems.emptySubsystems;

import frc.robot.subsystems.VisionSubsystem;

public class EmptyVisionSubsystem extends VisionSubsystem {
    public EmptyVisionSubsystem() {

    }

    @Override
    protected void constructor() {
        
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public double getDistance() {
        return 0;
    }

    @Override
    public double getAngle() {
        return 0;
    }

    @Override
    public boolean targetDetected() {
        return false;
    }

    @Override
    public double getOffset() {
        return 0;
    }
}