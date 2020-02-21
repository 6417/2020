package frc.robot.commands;

import java.util.function.Supplier;

import ch.team6417.lib.utils.LatchedBoolean;
import ch.team6417.lib.utils.LatchedBoolean.EdgeDetection;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.ColorDetected;

public class IsConnected extends CommandBase {
  private final ControlPanelSubsystem m_subsystem = ControlPanelSubsystem.getInstance();
  private ColorDetected cColor;
  private boolean finished = false;
  private LatchedBoolean overRotations = new LatchedBoolean(EdgeDetection.RISING);

  public IsConnected() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cColor = m_subsystem.getColor();
    ControlPanelSubsystem.getInstance().resetEncoder();

    overRotations = new LatchedBoolean(EdgeDetection.RISING);
    finished = false;
    m_subsystem.isConected = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (overRotations.update(Math.abs(m_subsystem.getEncoderValue()) >= 5120)) {
      finished = (cColor ==  m_subsystem.getColor());
      m_subsystem.isConected = (cColor !=  m_subsystem.getColor());
    } else if (RobotContainer.getSecurityMechanismsButton()) {
      finished = false;
      m_subsystem.isConected = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}