package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Operator Interface */
    public OperatorInterface oi = new OperatorInterface() {};

    /* Subsystems */
    public final PowerDistribution power = new PowerDistribution();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSubsystem vision = new VisionSubsystem();

    /* Path follower */
    private SendableChooser<Command> autoChooser;

    // private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a 
     * parameter. The device will be automatically initialized with default 
     * parameters.
     */
   // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

   

  /* Test System */
  //  private TestChecklist m_test;

  /* Cameras */
  // public UsbCamera cam0;

  // public static Map<String, Trajectory> trajectoryList = new HashMap<String, Trajectory>();
  // public static Map<String, List<PathPlannerTrajectory>> pptrajectoryList =
  //     new HashMap<String, List<PathPlannerTrajectory>>();
  // public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

  private static RobotContainer instance;

  public static RobotContainer GetInstance() {
    return instance;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    instance = this;

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    configureAutoPaths();
    configureAutoCommands();
    drivetrain.registerTelemetry(logger::telemeterize);

    // cam0 = CameraServer.startAutomaticCapture(0);
    // cam0.setConnectVerbose(0);

    ShuffleboardTab tab = Shuffleboard.getTab("MAIN");
    tab.add(autoChooser).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
    tab.addNumber("DriveTrain/Drive Scaling", () -> oi.driveScalingValue());
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    configureButtonBindings();

    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          drive.withVelocityX(oi.getTranslateX()) // Drive forward with negative Y (forward)
              .withVelocityY(oi.getTranslateY()) // Drive left with negative X (left)
              .withRotationalRate(oi.getRotate()) // Drive counterclockwise with negative X (left)
          )
    );
    vision.setDefaultCommand(new PhotonVisionCommand(vision, drivetrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // x-stance
    oi.getXStanceButton().whileTrue(drivetrain.applyRequest(() -> brake));

    oi.getRobotRelative().whileTrue(drivetrain.applyRequest(() -> 
      point.withModuleDirection(new Rotation2d(oi.getTranslateX(), oi.getTranslateY()))));

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    oi.getStartButton().and(oi.getYButton()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    oi.getStartButton().and(oi.getXButton()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    oi.getBackButton().and(oi.getYButton()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    oi.getBackButton().and(oi.getXButton()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

    oi.getStartButton().and(oi.getAButton()).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    oi.getStartButton().and(oi.getBButton()).whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    // Add commands to Autonomous Sendable Chooser
    autoChooser = AutoBuilder.buildAutoChooser("Test");
  }

  private void configureAutoPaths() {
    // NamedCommands.registerCommand("Intake", new IntakeCommand(shooter));
    // NamedCommands.registerCommand(
    //     "ElevateUp", new ElevateCommand(elevator, ElevatorConstants.kTargetElevatorHigh));
    // NamedCommands.registerCommand(
    //     "ElevateDown", new ElevateCommand(elevator, ElevatorConstants.kTargetElevatorLow));
    // NamedCommands.registerCommand(
    //     "RotateUp", new RotateWristCommand(wrist, WristConstants.kTargetWristHigh));
    // NamedCommands.registerCommand(
    //     "RotateDown", new RotateWristCommand(wrist, WristConstants.kTargetWristLow));
    // NamedCommands.registerCommand("Shoot", new ShootCommand(shooter));
    // NamedCommands.registerCommand("ShootSlow", new ShootSlowCommand(shooter));
  }

  public void simulationInit() {}

  public void simulationPeriodic() {
    // vision.simulationPeriodic(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()));

    // var debugField = vision.getSimDebugField();
    // debugField.getObject("EstimatedRobot").setPose(drivetrain.getPose());
    // debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.getModulePoses());

    // // Calculate battery voltage sag due to current draw
    // var batteryVoltage =
    //         BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.getCurrentDraw());

    // // Using max(0.1, voltage) here isn't a *physically correct* solution,
    // // but it avoids problems with battery voltage measuring 0.
    // RoboRioSim.setVInVoltage(Math.max(0.1, batteryVoltage));
  }

  public void testInit() {
  }

  public void testPeriodic() {
  }

  public void testExit() {
  }
}
