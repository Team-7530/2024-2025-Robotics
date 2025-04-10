package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class GetCoralPositionCommand extends SequentialCommandGroup {
  /**
   * Sets arm and wrist to collect coral
   * @param arm Subsystem
   * @param wrist Subsystem
   */
  public GetCoralPositionCommand(ArmSubsystem arm, WristSubsystem wrist) {
    addCommands(
        arm.armToPositionCommand(0.25),
        wrist.wristToPositionCommand(ScoringConstants.LoadWristPosition));
  }
}
