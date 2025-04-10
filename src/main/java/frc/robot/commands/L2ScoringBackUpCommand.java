package frc.robot.commands;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class L2ScoringBackUpCommand extends Command {

  private final PathConstraints pathConstraints;
  private final CommandSwerveDrivetrain drivetrain;
  private Command pathCommand;

  public L2ScoringBackUpCommand(CommandSwerveDrivetrain drivetrain) {
    setName("L2ScoringBackUpCommand");
    this.drivetrain = drivetrain;
    this.pathConstraints =
        new PathConstraints(1.0, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drivetrain.getState().Pose;

    // The rotation component in these poses represents the direction of travel
    Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    Pose2d endPos =
        new Pose2d(
            currentPose
                .getTranslation()
                .plus(
                    new Translation2d(
                        ScoringConstants.L2BackupAmountX, ScoringConstants.L2BackupAmountY)),
            new Rotation2d());

    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(startPos, endPos),
            pathConstraints,
            null, // Ideal starting state can be null for on-the-fly paths
            new GoalEndState(0.0, currentPose.getRotation()));
    path.preventFlipping = true;

    pathCommand = AutoBuilder.followPath(path);
  }

  @Override
  public void execute() {
    pathCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
