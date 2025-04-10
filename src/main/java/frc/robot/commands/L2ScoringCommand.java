package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class L2ScoringCommand extends SequentialCommandGroup {
  /**
   * Sets arm and wrist to L2 position
   * @param arm Subsystem
   * @param wrist Subsystem
   */
  public L2ScoringCommand(RobotContainer rc) {
    setName("L2ScoringCommand");
    addCommands(
        new L2ScoringPositionCommand(rc.arm, rc.wrist),
        new L2ScoringBackUpCommand(rc.drivetrain),
        rc.intake.outtakeL2Command());
  }
}
