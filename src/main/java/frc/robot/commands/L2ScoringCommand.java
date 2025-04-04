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
    addCommands(
        new L2ScoringPositionCommand(rc.arm, rc.wrist).withName("L2ScoringPositionCommand"),
        new L2ScoringBackUpCommand(rc.drivetrain).withName("L2ScoringBackUpCommand"),
        new OuttakeCommand(rc.intake).withName("OuttakeCommand"));
  }
}
