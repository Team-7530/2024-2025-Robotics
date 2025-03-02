package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class GetCoralCommand extends SequentialCommandGroup {

  public GetCoralCommand(ArmSubsystem arm, WristSubsystem wrist) {
    addCommands( 
      new ArmToPositionCommand(arm, 0.25)
          .withName("ArmToLoadingPosition"),
        new WristToPositionCommand(wrist, ScoringConstants.LoadWristPosition, false)
            .withName("WristToLoadingPosition"));
  }
}
