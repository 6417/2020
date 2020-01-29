package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.emptySubsystems.EmptyBallTransportSubsystem;

public class BallTransportSubsystem extends SubsystemBase {
    private WPI_TalonSRX transportMotor;
    private static BallTransportSubsystem mInstance;

    protected BallTransportSubsystem() {
        consuctor();
    }

    public static BallTransportSubsystem getInstance() {
        if (Constants.BALL_TRANSPORT_SUBSYSTEM_ENABLED) {
            if (mInstance == null) {
                mInstance = new BallTransportSubsystem();
                return mInstance;
            } else {
                return mInstance;
            }
        } else {
            return new EmptyBallTransportSubsystem();
        }
    }

    protected void consuctor() {
        transportMotor = new WPI_TalonSRX(Constants.BALL_TRANSPORT_MOTOR_CAN_ID);
    }

    public void setTransportMotor(double speed) {
        transportMotor.set(speed);
    }

    public void stopTransportMotor() {
        transportMotor.stopMotor();
    }

    public boolean getSensor() {
        return transportMotor.isFwdLimitSwitchClosed() == 1;
    }
}