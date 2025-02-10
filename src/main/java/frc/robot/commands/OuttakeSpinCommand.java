package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class OuttakeSpinCommand extends Command {
  private final ArmSubsystem armSubsystem;

  private boolean m_isFinished = false;
  private int m_counter = 0;

  public OuttakeSpinCommand(ArmSubsystem arm) {

    this.armSubsystem = arm;

    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    m_counter = 25;
    m_isFinished = false;

    armSubsystem.intakeOutSpin();
    System.out.println("Intake - SpinOut");
  }

  @Override
  public void execute() {
    m_isFinished = (--m_counter <= 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
