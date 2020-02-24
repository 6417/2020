package frc.robot.subsystems.emptySubsystems;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.subsystems.ClimberSubsystem;

public class EmptyClimberSubsystem extends ClimberSubsystem {
    public EmptyClimberSubsystem() {

    }

    @Override
    protected void constructor() {
        
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void climb() {
        climberPIDRight.setReference(0, ControlType.kVelocity);
        climberPIDLeft.setReference(0, ControlType.kVelocity);
    }

    @Override
    public void stopClimber() {
        
    }

    @Override
    public void setHeight(int encoderTicks) {
        
    }

    @Override
    public void resetHeight() {
        
    }

    @Override
    public void resetLeftHeight() {
        
    }

    @Override
    public void resetRightHeight() {
        
    }

    @Override
    public double getHeight() {
        return 0;
    }

    @Override
    public double calculatePositionDifference() {
        return 0;
    }

    @Override
    public void checkSafetyStop() {
        
    }

    @Override
    public void checkLimits() {
        
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        
    }
}