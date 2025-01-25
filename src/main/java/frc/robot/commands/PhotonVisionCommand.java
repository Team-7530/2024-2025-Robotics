package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem; 

public class PhotonVisionCommand extends Command {
  private final VisionSubsystem vision;
  private final CommandSwerveDrivetrain drivetrain;
  private Field2d field2d = new Field2d(); 

  public PhotonVisionCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
    this.vision = vision;
    this.drivetrain = drivetrain;

    addRequirements(vision);
  }

  @Override
  public void initialize() {
    ShuffleboardTab tab = Shuffleboard.getTab("MAIN");
    tab.add(field2d).withSize(6, 4).withWidget(BuiltInWidgets.kField);
  }

  @Override
  public void execute() {
    // Correct pose estimate with vision measurements
    var visionEst = vision.getEstimatedGlobalPose();
    
    visionEst.ifPresent(est -> {
      // Change our trust in the measurement based on the tags we can see

      var estStdDevs = vision.getEstimationStdDevs();

      drivetrain.addVisionMeasurement(
          est.estimatedPose.toPose2d(),
            Utils.fpgaToCurrentTime(est.timestampSeconds),
            estStdDevs);   
    });
    field2d.setRobotPose(drivetrain.getState().Pose);
  }

}