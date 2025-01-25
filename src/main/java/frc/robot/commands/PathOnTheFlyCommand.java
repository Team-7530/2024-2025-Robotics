package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathOnTheFlyCommand extends Command {
  
        private final Pose2d targetPose;
        // private final CommandSwerveDrivetrain drivetrain;
        private PathConstraints pathConstraints;

          /** Creates a new driveToPose. */
  public PathOnTheFlyCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        // Pose2d targetPose = new Pose2d(1, 1, Rotation2d.fromDegrees(180));

    this.targetPose = targetPose;
//     this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathConstraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  }

  @Override
  public void execute() {
    AutoBuilder.pathfindToPose(
        targetPose,
        pathConstraints,
        0.0).schedule();

        //     // Create the path using the waypoints created above
//     PathPlannerPath path = new PathPlannerPath(
//             waypoints,
//             constraints,
//             null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
//             new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
//     );

    // Prevent the path from being flipped if the coordinates are already correct
//     path.preventFlipping = true;
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  

}