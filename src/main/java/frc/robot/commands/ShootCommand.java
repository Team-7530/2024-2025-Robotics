package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ShootCommand extends Command {
  private final ArmSubsystem m_shooter;

  private boolean m_isFinished = false;
  private int m_shootCounter = 0;

  public ShootCommand(ArmSubsystem shooter) {

    this.m_shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shootCounter = 25;
    m_isFinished = false;

    m_shooter.intakeOut();
    System.out.println("Shooting");
  }

  @Override
  public void execute() {
    m_isFinished = (--m_shootCounter <= 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
