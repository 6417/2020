/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.Level;

import ch.team6417.lib.utils.LatchedBoolean;
import ch.team6417.lib.utils.LatchedBoolean.EdgeDetection;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.extern.java.Log;

@Log
public class PneumaticSubsystem extends SubsystemBase {

  private Compressor compressor = new Compressor(Constants.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID);
  private DoubleSolenoid liftSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID,
      Constants.PNEUMATIC_SUBSYSTEM_LIFT_SOLENOID_EXTEND_ID,
      Constants.PNEUMATIC_SUBSYSTEM_LIFT_SOLENOID_RETRACT_ID);

  private DoubleSolenoid bumperSolenoid = new DoubleSolenoid(Constants.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID,
      Constants.PNEUMATIC_SUBSYSTEM_BUMPER_SOLENOID_EXTEND_ID,
      Constants.PNEUMATIC_SUBSYSTEM_BUMPER_SOLENOID_RETRACT_ID);

  private LatchedBoolean pressureTankFull = new LatchedBoolean(EdgeDetection.FALLING);

  private static PneumaticSubsystem mInstance;

  /**
   * Creates a new ExampleSubsystem.
   */

  public enum PneumaticState {
    OFF, FORWARD, REVERSE
  }
  
  private PneumaticSubsystem() {
    constructor();
  }

  protected void constructor() {
    SendableRegistry.addChild(this, liftSolenoid);
    SendableRegistry.addChild(this, bumperSolenoid);
    // SendableRegistry.addChild(this, compressor);
    SendableRegistry.setName(liftSolenoid, "Lift");
    SendableRegistry.setName(bumperSolenoid, "Bumper");
    // SendableRegistry.setName(compressor, "Compressor");

    compressor.setClosedLoopControl(false);
  }

  @Override
  public void periodic() {
    if (compressor.getCompressorNotConnectedFault()) {
      log.log(Level.SEVERE, "Compressor not connected!");
    }
    if (compressor.getCompressorCurrentTooHighFault()) {
      log.log(Level.SEVERE, "Compressor current too high!");
    }
    if (compressor.getCompressorShortedFault()) {
      log.log(Level.SEVERE, "Compressor shorted!");
    }

    if (pressureTankFull.update(compressor.getPressureSwitchValue())) {
      log.log(Level.INFO, "Pressure Tank full!");
    }
  }

  public static PneumaticSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new PneumaticSubsystem();
      return mInstance;
    } else {
      return mInstance;
    }
  }

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

  public boolean extendLift() {
    set(liftSolenoid, PneumaticState.FORWARD);
//    set(liftSolenoid, PneumaticState.OFF);
    return true;
  }

  public void retractLift() {
    set(liftSolenoid, PneumaticState.REVERSE);
//    set(liftSolenoid, PneumaticState.OFF);
  }

  public void closeLift() {
    set(liftSolenoid, PneumaticState.OFF);
  }

  public void extendBumper() {
    set(bumperSolenoid, PneumaticState.FORWARD);
  }

  public boolean retractBumper() {
    set(bumperSolenoid, PneumaticState.REVERSE);
//    set(bumperSolenoid, PneumaticState.OFF);
    return true;
  }

  public boolean closeBumper() {
    set(bumperSolenoid, PneumaticState.OFF);
    return true;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addBooleanProperty("Compressor connected", () -> compressor.getCompressorNotConnectedFault(), null);
    builder.addBooleanProperty("Compressor current too high", () -> compressor.getCompressorCurrentTooHighFault(), null);
  }
}
