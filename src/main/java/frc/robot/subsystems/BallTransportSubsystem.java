package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.emptySubsystems.EmptyBallTransportSubsystem;

public class BallTransportSubsystem extends SubsystemBase {
    private WPI_TalonSRX transportMotor;
    private static BallTransportSubsystem mInstance;
    private DigitalInput sensor;

    protected BallTransportSubsystem() {
        constructor();
    }

    public static BallTransportSubsystem getInstance() {
        if (mInstance == null) {
            if (Constants.BALL_TRANSPORT_SUBSYSTEM_ENABLED) {
                mInstance = new BallTransportSubsystem();
            } else {
                mInstance = new EmptyBallTransportSubsystem();
            }
        }
        return mInstance;
    }

    protected void constructor() {
        transportMotor = new WPI_TalonSRX(Constants.BALL_TRANSPORT_MOTOR_CAN_ID);
        sensor = new DigitalInput(Constants.BALL_TRANSPORT_SENSOR_DIO_PORT);
    }

    public void setTransportMotor(double speed) {
        transportMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopTransportMotor() {
        System.out.println("stop");
        transportMotor.stopMotor();
    }

    public double getPercents() {
        return(transportMotor.get());
    } 

    public boolean getSensor() {
        return !sensor.get();
    }
}