package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {
  private final ClimberSubsystem m_climb;

  /**
   * Closes clamp and moves to raised position
   * @param climb Subsystem
   */
  public ClimbCommand(ClimberSubsystem climb) {
    this.m_climb = climb;

    addRequirements(climb);
  }

  @Override
  public void initialize() {
    m_climb.setClamp(true);
    m_climb.climb();
  }

  @Override
  public boolean isFinished() {
    return m_climb.isAtFullClimbPosition();
  }
}
