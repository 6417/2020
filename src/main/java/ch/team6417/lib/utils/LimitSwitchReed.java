package ch.team6417.lib.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

public class LimitSwitchReed {
    public enum LimitSwitchPort {
        FORWARD, REVERSE
    }

    LimitSwitchPort port;
    WPI_TalonSRX motorControllerTalon;
    CANSparkMax motorControllerSpark;

    LimitSwitchReed(WPI_TalonSRX motorController, LimitSwitchPort port) {
        this.motorControllerTalon = motorController;
        this.port = port;
    }

    LimitSwitchReed(CANSparkMax motorController, LimitSwitchPort port) {
        this.motorControllerSpark = motorController;
        this.port = port;
    }

    boolean getReed() throws Exception {
        if (motorControllerTalon != null) {
            switch (this.port) {
                case FORWARD:
                return motorControllerTalon.isFwdLimitSwitchClosed() == 1;
                case REVERSE:
                return motorControllerTalon.isRevLimitSwitchClosed() == 1;
                default:
                throw new Exception("Somthing with the Port went Wrong with the port: " + String.valueOf(port));
            }
        } else if (motorControllerSpark != null) {
            switch (port) {
            case FORWARD:
            CANDigitalInput digitalInputF = motorControllerSpark.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
            return digitalInputF.get();
            case REVERSE:
            CANDigitalInput digitalInputR = motorControllerSpark.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
            return digitalInputR.get();
            default:
            throw new Exception("Somthing with the Port went Wrong with the port: " + String.valueOf(port));
            }
        }
        
}