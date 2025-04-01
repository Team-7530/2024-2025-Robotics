package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristSpeedCommand extends Command {
  private final WristSubsystem m_wrist;
  private double m_speed = 0.0;

  public WristSpeedCommand(WristSubsystem wrist, double speed) {
    this.m_wrist = wrist;
    this.m_speed = speed;

    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    m_wrist.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
