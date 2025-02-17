package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {
  private final ClimberSubsystem m_climb;

  private boolean m_isFinished = false;

  public ClimbCommand(ClimberSubsystem climb, boolean closed) {
    this.m_climb = climb;

    addRequirements(climb);
  }

  @Override
  public void initialize() {
    m_isFinished = false;
    m_climb.setClamp(true);
    m_climb.climb();
  }

  @Override
  public void execute() {
    m_isFinished = m_climb.isFullClimbPosition();
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
