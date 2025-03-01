package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ClimbPositionCommand extends SequentialCommandGroup {

  public ClimbPositionCommand(ArmSubsystem arm, WristSubsystem wrist) {
    addCommands(
        new ArmToPositionCommand(arm, ScoringConstants.ClimbArmPosition)
            .withName("ArmToLoadingPosition"),
        new WristToPositionCommand(wrist, ScoringConstants.ClimbWristPosition, true)
            .withName("WristToLoadingPosition"));
  }
}
