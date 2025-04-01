package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSpeedCommand extends Command {
  private final ArmSubsystem m_arm;
  private double m_speed = 0.0;

  public ArmSpeedCommand(ArmSubsystem arm, double speed) {
    this.m_arm = arm;
    this.m_speed = speed;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    m_arm.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
