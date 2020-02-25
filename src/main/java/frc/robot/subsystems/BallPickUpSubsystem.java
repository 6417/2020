package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
    private DoubleSolenoid protectCylinder;
    public boolean protectorExtended = false;

    protected BallPickUpSubsystem() {
        constructor();
    }

    protected void constructor() {
        pickUpMotor = new WPI_TalonSRX(Constants.BALL_PICKUP_MOTOR_CAN_ID);
        pickUpMotor.setInverted(InvertType.InvertMotorOutput);
        PickupCylinder = new DoubleSolenoid(Constants.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID, Constants.BALL_PICKUP_SUBSYSTEM_EXTEND_ID, Constants.BALL_PICKUP_SUBSYSTEM_RETRACT_ID);
        protectCylinder = new DoubleSolenoid(Constants.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID, Constants.BALL_PICKUP_PROTECTER_EXTEND_ID, Constants.BALL_PICKUP_PROTECTER_RETRACT_ID);
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
                break;
            case REVERSE:
                PickupCylinder.set(Value.kReverse);
                break;
            case OFF:
                PickupCylinder.set(Value.kOff);
                break;
        }

    }

    public void setPickUpMotor(double speed) {
        pickUpMotor.set(speed);
    }

    public void stopPickUpMotor() {
        pickUpMotor.stopMotor();
    }

    public void setProtectCylinder(PneumaticState state) {
        switch(state) {
            case FORWARD:
                protectCylinder.set(Value.kForward);
                break;
            case REVERSE:
                protectCylinder.set(Value.kReverse);
                break;
            case OFF:
                protectCylinder.set(Value.kOff);
                break;
        }
    }
}