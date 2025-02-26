package frc.robot.commands;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveTeleopCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final OperatorInterface oi;

  private final SwerveRequest.FieldCentric driveFieldCentric =
      new SwerveRequest.FieldCentric()
          .withDeadband(DriveTrainConstants.maxSpeed * 0.1)
          .withRotationalDeadband(DriveTrainConstants.maxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.RobotCentric driveRobotCentric =
      new SwerveRequest.RobotCentric()
          .withDeadband(DriveTrainConstants.maxSpeed * 0.1)
          .withRotationalDeadband(DriveTrainConstants.maxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public SwerveTeleopCommand(CommandSwerveDrivetrain drivetrain, OperatorInterface oi) {
    this.drivetrain = drivetrain;
    this.oi = oi;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (oi.getRobotRelative().getAsBoolean()) {
      drivetrain.setControl(
          driveRobotCentric
              .withVelocityX(oi.getTranslateX() * DriveTrainConstants.maxSpeed)
              .withVelocityY(oi.getTranslateY() * DriveTrainConstants.maxSpeed)
              .withRotationalRate(oi.getRotate() * DriveTrainConstants.maxAngularRate));
    } else {
      drivetrain.setControl(
          driveFieldCentric
              .withVelocityX(oi.getTranslateX() * DriveTrainConstants.maxSpeed)
              .withVelocityY(oi.getTranslateY() * DriveTrainConstants.maxSpeed)
              .withRotationalRate(oi.getRotate() * DriveTrainConstants.maxAngularRate));
    }
  }
}
