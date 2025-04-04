package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristToPositionCommand extends Command {
  private final WristSubsystem m_wrist;
  private double m_targetPosition = 0.0;
  private boolean m_isFinished = false;

  /**
   * Moves the wrist to a target position
   * @param wrist Subsystem
   * @param position Position between 0 and 1
   * @param slow bool
   */
  public WristToPositionCommand(WristSubsystem wrist, double position, boolean slow) {
    this.m_wrist = wrist;
    this.m_targetPosition = position;

    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    m_isFinished = false;
    m_wrist.setPosition(m_targetPosition);
  }

  @Override
  public void execute() {
    if (!m_isFinished)
      m_isFinished = m_wrist.isAtPosition();
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
