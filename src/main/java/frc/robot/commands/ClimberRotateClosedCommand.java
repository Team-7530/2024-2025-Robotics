// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ClimberSubsystem;

// public class ClimberRotateClosedCommand extends Command {
//   private final ClimberSubsystem m_climb;

//   private boolean m_isFinished = false;
//   private int m_Counter = 0;

//   public ClimberRotateClosedCommand(ClimberSubsystem climb) {
//     this.m_climb = climb;
//     addRequirements(climb);
//   }

//   @Override
//   public void initialize() {
//     m_isFinished = false;
//     m_Counter = 500;
//     m_climb.rotateClosed();
//   }

//   @Override
//   public void execute() {
//     m_isFinished = --m_Counter <= 0;
//   }

//   @Override
//   public boolean isFinished() {
//     return m_isFinished;
//   }
// }
