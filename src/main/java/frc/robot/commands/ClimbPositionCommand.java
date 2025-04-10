package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ClimbPositionCommand extends SequentialCommandGroup {

  /**
   * Sets the arm and the wrist to their location during climb
   * @param arm Subsystem 
   * @param wrist Subsystem 
   */
  public ClimbPositionCommand(ArmSubsystem arm, WristSubsystem wrist) {
    setName("ClimbPositionCommand");
    addCommands(
        new ParallelCommandGroup(
          arm.armToPositionCommand(ScoringConstants.ClimbArmPosition),
          wrist.wristToPositionCommand(ScoringConstants.ClimbWristPosition)),
        Commands.runOnce(() -> wrist.stop()),
        Commands.runOnce(() -> arm.stop())
    );
  }
}
