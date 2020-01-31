package frc.robot.subsystems.emptySubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallTransportSubsystem;

public class EmptyBallTransportSubsystem extends BallTransportSubsystem {
    public EmptyBallTransportSubsystem() {
        super();
    }

    protected void constructor() {
        
    }

    public void setTransportMotor(double speed) {
        
    }

    public void stopTransportMotor() {
        
    }

    public boolean getSensor() {
        return true;
    }

    @Override
    public double getPercents() {
        return 0;
    }
}