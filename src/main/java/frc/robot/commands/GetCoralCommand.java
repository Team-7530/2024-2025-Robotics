package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class GetCoralCommand extends ParallelCommandGroup {

  public GetCoralCommand(ArmSubsystem arm, WristSubsystem wrist) {
    addCommands(
        new ArmToPositionCommand(arm, ScoringConstants.LoadArmPosition)
            .withName("ArmToLoadingPosition"),
        new WristToPositionCommand(wrist, ScoringConstants.LoadWristPosition, false)
            .withName("WristToLoadingPosition"));
  }
}
