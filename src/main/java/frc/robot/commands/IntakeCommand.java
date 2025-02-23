package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class IntakeCommand extends Command {
  private final IntakeSubsystem m_IntakeSubsystem;

  private boolean m_isFinished = false;
  private int m_IntakeCounter = 0;

  public IntakeCommand(IntakeSubsystem intakeSubsystem) {

    this.m_IntakeSubsystem = intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
  }

  @Override
  public void initialize() {
    m_IntakeCounter = 500;
    m_isFinished = false;

    m_IntakeSubsystem.intakeIn();
    System.out.println("Intake started");
  }

  @Override
  public void execute() {
    m_isFinished = (--m_IntakeCounter <= 0) || m_IntakeSubsystem.hasCoralLoaded();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.intakeStop();
    System.out.println("Intake stop");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
