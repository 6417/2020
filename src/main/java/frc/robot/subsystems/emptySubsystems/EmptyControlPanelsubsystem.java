package frc.robot.subsystems.emptySubsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.subsystems.ControlPanelSubsystem;

public class EmptyControlPanelSubsystem extends ControlPanelSubsystem {

    public EmptyControlPanelSubsystem() {
        super();
    }

    protected void constructor() {
        super.isConected = false;
    }

    public ColorDetected getColor() {
        return ColorDetected.NONE;
    }

    public boolean getReedLiftBotom() {
        return true;
    }

    public boolean getReedBumperFront() {
        return true;
    }

    public void setSensorPos(int pos) {

    }

    public void setMotorForRotations(int rotations) {

    }

    public double getEncoderValue() {
        return 0;
    }

    public boolean isMotorInRange() {
        return true;
    }

    public void stopMotor() {

    }


    @Override
    public void periodic() {
        
    }

    public void setMotor(double speed) {

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
    public void initSendable(SendableBuilder builder) {
        
    }
} 