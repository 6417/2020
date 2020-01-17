/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.Level;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import ch.team6417.lib.utils.LatchedBoolean;
import ch.team6417.lib.utils.LatchedBoolean.EdgeDetection;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.enums.PneumaticState;
import lombok.extern.java.Log;

@Log
public class PneumaticSubsystem extends SubsystemBase {

  private Compressor compressor = new Compressor(RobotContainer.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID);
  private DoubleSolenoid liftSolenoid = new DoubleSolenoid(RobotContainer.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID,
      RobotContainer.PNEUMATIC_SUBSYSTEM_LIFT_SOLENOID_EXTEND_ID,
      RobotContainer.PNEUMATIC_SUBSYSTEM_LIFT_SOLENOID_RETRACT_ID);

  private DoubleSolenoid bumperSolenoid = new DoubleSolenoid(RobotContainer.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID,
      RobotContainer.PNEUMATIC_SUBSYSTEM_LIFT_SOLENOID_EXTEND_ID,
      RobotContainer.PNEUMATIC_SUBSYSTEM_LIFT_SOLENOID_RETRACT_ID);


  private LatchedBoolean pressureTankFull = new LatchedBoolean(EdgeDetection.FALLING);
  /**
   * Creates a new ExampleSubsystem.
   */
  public PneumaticSubsystem() {

  }

  // @Override
  // public void periodic() {
  //   if(compressor.getCompressorNotConnectedFault()) {
  //     log.log(Level.SEVERE, "Compressor not connected!");
  //   }
  //   if (compressor.getCompressorCurrentTooHighFault()) {
  //     log.log(Level.SEVERE, "Compressor current too high!");
  //   }
  //   if(compressor.getCompressorShortedFault()) {
  //     log.log(Level.SEVERE, "Compressor shorted!");
  //   }

  //   if(pressureTankFull.update(compressor.getPressureSwitchValue()))
  //   {
  //     log.log(Level.INFO, "Pressure Tank full!");
  //   }
  // }
  
  void set(DoubleSolenoid cylinder, PneumaticState state) {
    switch (state) {
      case OFF:
        cylinder.set(Value.kOff);
        break;
      
      case FORWARD:
        cylinder.set(Value.kForward);
        break;

      case REVERSE:
        cylinder.set(Value.kReverse);
        break;
    }
  }

  boolean getRead(WPI_TalonSRX motorController, int port) throws Exception{
    switch (port) {
      case 1:
        return motorController.isFwdLimitSwitchClosed() == 1;
      case 0:
        return motorController.isRevLimitSwitchClosed() == 1;
      default:
        throw new Exception("Port must be 1(forward) or 0(reverse) not " + String.valueOf(port));
    }
  }
}
