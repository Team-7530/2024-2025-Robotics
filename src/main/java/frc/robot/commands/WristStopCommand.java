package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristStopCommand extends Command {
  private final WristSubsystem m_wrist;

  public WristStopCommand(WristSubsystem wrist) {
    this.m_wrist = wrist;

    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    m_wrist.stop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
