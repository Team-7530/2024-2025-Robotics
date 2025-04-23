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
        rc.armwrist.l2ScoringPositionCommand(),
        rc.armwrist.intake.outtakeL2Command());
  }
}
