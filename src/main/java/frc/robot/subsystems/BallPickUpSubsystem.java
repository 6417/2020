package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelSubsystem.PneumaticState;
import frc.robot.subsystems.emptySubsystems.EmptyBallPickUpSubsystem;

public class BallPickUpSubsystem extends SubsystemBase {
    private static BallPickUpSubsystem mInstance;
    private WPI_TalonSRX pickUpMotor;
    private DoubleSolenoid PickupCylinder;

    protected BallPickUpSubsystem() {
        constructor();
    }

    protected void constructor() {
        pickUpMotor = new WPI_TalonSRX(Constants.BALL_PICKUP_MOTOR_CAN_ID);
        PickupCylinder = new DoubleSolenoid(Constants.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID, Constants.BALL_PICKUP_SUBSYSTEM_EXTEND_ID, Constants.BALL_PICKUP_SUBSYSTEM_RETRACT_ID);
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

    public void setPickupCylinder(PneumaticState state){
        switch(state){
            case FORWARD:
                PickupCylinder.set(Value.kForward);
            case REVERSE:
                PickupCylinder.set(Value.kReverse);
        }

    }

    public void setPickUpMotor(double speed) {
        pickUpMotor.set(speed);
    }

    public void stopPickUpMotor() {
        pickUpMotor.stopMotor();
    }
}