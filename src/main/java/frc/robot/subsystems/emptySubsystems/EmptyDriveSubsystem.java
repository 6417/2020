package frc.robot.subsystems.emptySubsystems;

import frc.robot.subsystems.DriveSubsystem;

public class EmptyDriveSubsystem extends DriveSubsystem {
    public EmptyDriveSubsystem() {
        super();
    } 

    protected void constructor() {

    }

    public void driveLeft(double l_percentage) {

    }

    public void driveRight(double r_percentage) {

    }

    public void drive(double xSpeed, double ySpeed) {

    }

    public void drive() {

    }

    public void stopDrive() {

    }

    @Override
    public double getEncoderLeft() {
        return 0;
    }

    @Override
    public double getEncoderRight() {
        return 0;
    }

    @Override
    public void resetEncoders() {
        
    }

    @Override
    public void periodic() {
        
    }
}