package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPositionCommand extends Command {
  private final ArmSubsystem m_arm;

  private double m_targetPosition = 0.0;
  private boolean m_isFinished = false;

  public ArmToPositionCommand(ArmSubsystem arm, double position) {
    this.m_arm = arm;
    this.m_targetPosition = position;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    m_isFinished = false;
    m_arm.setArmPosition(m_targetPosition);
  }

  @Override
  public void execute() {
    m_isFinished = Math.abs(m_arm.getArmPosition() - m_targetPosition) < 0.05;
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
