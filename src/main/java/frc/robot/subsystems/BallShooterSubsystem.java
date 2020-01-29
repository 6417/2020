package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.emptySubsystems.EmptyBallShooterSubsystem;

public class BallShooterSubsystem extends SubsystemBase {
    private static BallShooterSubsystem mInstance;
    private CANSparkMax loader;
    private CANSparkMax shooterMaster;
    private CANSparkMax shooterRight;
    private CANEncoder masterEncoder;
    public double shooterSpeed;

    protected BallShooterSubsystem() {
        constructor();
    }

    protected void constructor() {
        loader = new CANSparkMax(Constants.BALL_SHOOTER_SUBSYSTEM_LOADER_CAN_ID, MotorType.kBrushless);
        loader.setInverted(true);

        shooterMaster = new CANSparkMax(Constants.BALL_SHOOTER_SUBSYSTEM_SHOOTER_LEFT_CAN_ID,
            MotorType.kBrushless);
        shooterRight = new CANSparkMax(Constants.BALL_SHOOTER_SUBSYSTEM_SHOOTER_RIGHT_CAN_ID,
            MotorType.kBrushless);

        shooterRight.follow(shooterMaster, true);
        masterEncoder = shooterMaster.getEncoder();
    }

    public static BallShooterSubsystem getInstance() {
        if (Constants.BALL_SHOOTER_SUBSYSTEM_ENABLED) {
            if (mInstance == null) {
                mInstance = new BallShooterSubsystem();
                return mInstance;
            } else {
                return mInstance;
            }
        } else {
            return new EmptyBallShooterSubsystem();
        }
    }

    public void setShooter(double speed) {
        shooterMaster.set(speed);
    }

    public void setLoader(double speed) {
        loader.set(speed);
    }

    public void stopShooter() {
        shooterMaster.stopMotor();
    }

    public void stopLoader() {
        loader.stopMotor();
    }

    public double getSpeed(){
        double shooterSpeed = masterEncoder.getVelocity();
        return(shooterSpeed);
    }
    

}