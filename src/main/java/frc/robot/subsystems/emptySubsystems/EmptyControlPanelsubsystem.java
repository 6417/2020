package frc.robot.subsystems.emptySubsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.subsystems.ControlPanelSubsystem;

public class EmptyControlPanelSubsystem extends ControlPanelSubsystem {
    public EmptyControlPanelSubsystem() {
        super();
    }

    protected void constructor() {
    }

    public ColorDetected getColor() {
        return ColorDetected.NONE;
    }

    public boolean getReedLiftBotom() {
        return false;
    }

    public boolean getReedBumperFront() {
        return false;
    }

    public void setSensorPos(int pos) {

    }

    public void setMotorForRotations(int rotations) {

    }

    public int getEncoderValue() {
        return 0;
    }

    public boolean isMotorInRange() {
        return true;
    }

    public void stopMotor() {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
} 