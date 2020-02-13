/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import ch.team6417.lib.utils.LatchedBoolean;
import ch.team6417.lib.utils.LatchedBoolean.EdgeDetection;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.emptySubsystems.EmptyPneumaticSubsystem;
import lombok.extern.java.Log;

@Log
public class PneumaticSubsystem extends SubsystemBase {

  private Compressor compressor;

  private LatchedBoolean pressureTankFull;

  private static PneumaticSubsystem mInstance;

  /**
   * Creates a new ExampleSubsystem.
   */
  
  protected PneumaticSubsystem() {
    constructor();
  }

  protected void constructor() {
    compressor = new Compressor(Constants.PNEUMATIC_SUBSYSTEM_COMPRESSOR_CAN_ID);
  
    pressureTankFull = new LatchedBoolean(EdgeDetection.FALLING);


    // SendableRegistry.setName(compressor, "Compressor");

  }

  @Override
  public void periodic() {
    //   if (compressor.getCompressorNotConnectedFault()) {
    //    log.log(Level.SEVERE, "Compressor not connected!");
    //   }
    //   if (compressor.getCompressorCurrentTooHighFault()) {
    //    log.log(Level.SEVERE, "Compressor current too high!");
    //  }
    //   if (compressor.getCompressorShortedFault()) {
    //    log.log(Level.SEVERE, "Compressor shorted!");
    //  }

    //  if (pressureTankFull.update(compressor.getPressureSwitchValue())) {
    //    log.log(Level.INFO, "Pressure Tank full!");
    //  }
  }

  public static PneumaticSubsystem getInstance() {
    if (mInstance == null) {
      if (Constants.PNEUMATIC_SUBSYSTEM_ENABLED) {
        mInstance = new PneumaticSubsystem();
      } else {
        mInstance = new EmptyPneumaticSubsystem();
      }
    }
    return mInstance;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addBooleanProperty("Compressor connected", () -> compressor.getCompressorNotConnectedFault(), null);
    builder.addBooleanProperty("Compressor current too high", () -> compressor.getCompressorCurrentTooHighFault(), null);
  }
}