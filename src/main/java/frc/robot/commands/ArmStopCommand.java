package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmStopCommand extends Command {
  private final ArmSubsystem m_arm;

  public ArmStopCommand(ArmSubsystem arm) {
    this.m_arm = arm;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    m_arm.stop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
