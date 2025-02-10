package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class OuttakeCommand extends Command {
  private final IntakeSubsystem m_IntakeSubsystem;

  private boolean m_isFinished = false;
  private int m_counter = 0;

  public OuttakeCommand(IntakeSubsystem intake) {

    this.m_IntakeSubsystem = intake;

    addRequirements(m_IntakeSubsystem);
  }

  @Override
  public void initialize() {
    m_counter = 25;
    m_isFinished = false;

    m_IntakeSubsystem.intakeOut();
    System.out.println("Intake - Out");
  }

  @Override
  public void execute() {
    m_isFinished = (--m_counter <= 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
