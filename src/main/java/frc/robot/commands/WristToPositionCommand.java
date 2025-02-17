package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristToPositionCommand extends Command {
  private final WristSubsystem m_wrist;

  private double m_targetPosition = 0.0;
  private boolean m_isFinished = false;

  public WristToPositionCommand(WristSubsystem wrist, double position) {
    this.m_wrist = wrist;
    this.m_targetPosition = position;

    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    m_isFinished = false;
    m_wrist.setWristPosition(m_targetPosition);
  }

  @Override
  public void execute() {
    m_isFinished = Math.abs(m_wrist.getWristPosition() - m_targetPosition) < 0.01;
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
