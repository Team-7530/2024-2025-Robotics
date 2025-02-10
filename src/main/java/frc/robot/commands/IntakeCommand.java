package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends Command {
  private final ArmSubsystem m_ArmSubsystem;

  private boolean m_isFinished = false;
  private int m_IntakeCounter = 0;

  public IntakeCommand(ArmSubsystem armSubsystem) {

    this.m_ArmSubsystem = armSubsystem;

    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    m_IntakeCounter = 25;
    m_isFinished = false;

    m_ArmSubsystem.intakeIn();
    System.out.println("Intake started");
  }

  @Override
  public void execute() {
    m_isFinished = (--m_IntakeCounter <= 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.intakeStop();
    System.out.println("Intake stop");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
