package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.emptySubsystems.EmptyBallPickUpSubsystem;

public class BallPickUpSubsystem extends SubsystemBase {
    private static BallPickUpSubsystem mInstance;
    private CANSparkMax pickUpMotor;

    protected BallPickUpSubsystem() {
        constructor();
    }

    protected void constructor() {
        pickUpMotor = new CANSparkMax(Constants.BALL_PICKUP_MOTOR_CAN_ID, MotorType.kBrushed);
    }

    public static BallPickUpSubsystem getInstance() {
        if (mInstance == null) {
            if (Constants.BALL_PICKUP_SUBSYSTEM_ENABLED) {
                mInstance = new BallPickUpSubsystem();
            } else {
                mInstance = new EmptyBallPickUpSubsystem();
            }
        }
        return mInstance;
    }

    public void setPickUpMotor(double speed) {
        pickUpMotor.set(speed);
    }

    public void stopPickUpMotor() {
        pickUpMotor.stopMotor();
    }
}