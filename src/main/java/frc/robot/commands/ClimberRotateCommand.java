package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberRotateCommand extends Command {
  private final ClimberSubsystem m_climb;

  private boolean m_targetClose = false;
  private boolean m_isFinished = false;
  private int m_Counter = 0;

  public ClimberRotateCommand(ClimberSubsystem climb, boolean closed) {
    this.m_climb = climb;
    this.m_targetClose = closed;

    addRequirements(climb);
  }

  @Override
  public void initialize() {
    m_isFinished = false;
    m_Counter = 500;
    if (m_targetClose) m_climb.rotateClosed();
    else m_climb.rotateOpen();
  }

  @Override
  public void execute() {
    m_isFinished = --m_Counter <= 0;
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
