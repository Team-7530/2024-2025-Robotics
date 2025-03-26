package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberResetCommand extends Command {
  private final ClimberSubsystem m_climb;

  public ClimberResetCommand(ClimberSubsystem climb) {
    this.m_climb = climb;

    addRequirements(climb);
  }

  @Override
  public void initialize() {
    m_climb.setClamp(false);
    m_climb.restore();
  }

  @Override
  public boolean isFinished() {
    return m_climb.isAtRestoredPosition();
  }
}
