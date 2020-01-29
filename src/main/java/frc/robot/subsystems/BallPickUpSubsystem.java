package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.emptySubsystems.EmptyBallPickUpSubsystem;

public class BallPickUpSubsystem extends SubsystemBase {
    private static BallPickUpSubsystem mInstance;
    private WPI_TalonSRX pickUpMotor;

    protected BallPickUpSubsystem() {
        constructor();
    }

    protected void constructor() {
        pickUpMotor = new WPI_TalonSRX(Constants.BALL_PICKUP_MOTOR_CAN_ID);
    }

    public static BallPickUpSubsystem getInstance() {
        if (Constants.BALL_PICKUP_SUBSYSTEM_ENABLED) {
            if (mInstance == null) {
               mInstance = new BallPickUpSubsystem();
               return mInstance;
            } else {
                return mInstance;
            }
        } else {
            return new EmptyBallPickUpSubsystem();
        }
    }

    public void setPickUpMotor(double speed) {
        pickUpMotor.set(speed);
    }

    public void stopPickUpMotor() {
        pickUpMotor.stopMotor();
    }
}