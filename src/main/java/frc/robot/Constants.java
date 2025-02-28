package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.TunerConstants;

public final class Constants {

  public static final boolean COMPETITIONBOT = false;

  public static final boolean DEBUGGING = true;
  public static final boolean TESTING = false;

  public static final double TRIGGER_DEADBAND = 0.01;
  public static final double STICK_DEADBAND = 0.01;
  public static final double TRIGGER_SPEEDFACTOR = 0.5;

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  // public static final Matter CHASSIS =
  //     new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double TURN_CONSTANT = 0.75;

  public static final class Vision {

    public static final String LIMELIGHTNAME = "limelight";
    public static final String LIMELIGHTURL = "limelight.local";
    public static final String PHOTONVISIONURL = "photonvision.local";
    public static final String kCameraName1 = "OV9281";
    public static final String kCameraName2 = "OV9281-2";

    // Cam mounted - x = +toward front, 0 center, -toward rear in meters.
    //               y = +left of center, 0 center, -right of center in meters
    //               z = +up from base of robot in meters
    //              roll = rotate around front/rear in radians. PI = upsidedown
    //              pitch = tilt down/up along left/right axis. PI/4 = tilt down 45 degrees, -PI/4 =
    // tilt up 45
    //              yaw = rotate left/right around z axis. PI/4 = rotate camera to the left 45
    // degrees.
    public static final Transform3d kRobotToCam1 =
        new Transform3d(new Translation3d(0.2, -0.2, 0), new Rotation3d(0, -Math.PI * 0.25, 0));
    public static final Transform3d kRobotToCam2 =
        new Transform3d(new Translation3d(-0.2, 0.2, 0), new Rotation3d(0, 0, Math.PI));

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class DriveTrainConstants {
    // Maximum Speed - Meters per Second
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    // max angular velocity - Rotations per Second
    // 3/4 of a rotation per second
    public static final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  }

  public static final class AutoConstants {
    public static final PIDConstants translationPID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants rotationPID = new PIDConstants(5.0, 0.0, 0.0);
  }

  public static final class ScoringConstants {
    public static final double L1ArmPosition = 0.28;
    public static final double L1WristPosition = -0.3;

    public static final double L2ArmPosition = 0.25;
    public static final double L2WristPosition = -0.22;

    public static final double LoadArmPosition = 0.25;
    public static final double LoadWristPosition = -0.14;

    public static final double StowArmPosition = 0.43;
    public static final double StowWristPosition = 0.25;
  }

  public static final class ArmConstants {
    public static final String CANBUS = "CANFD";
    public static final int ARMMOTOR_ID = 31;
    public static final int ARMENCODER_ID = 32;

    public static final InvertedValue kArmInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kArmNeutralMode = NeutralModeValue.Brake;
    public static final SensorDirectionValue kArmEncoderDirection =
        SensorDirectionValue.Clockwise_Positive;
    public static final double kArmEncoderOffset = -0.015869;

    public static final double kArmChainRatio = 74.0 / 16.0;
    public static final double kArmGearboxRatio = 80.0; // 1:80
    public static final double kArmGearRatio =
        kArmChainRatio * kArmGearboxRatio; // chain ratio * Gearbox ratio

    public static final double armMotorKG = 0.5;
    public static final double armMotorKS = 0.0;
    public static final double armMotorKV = 0.0;
    public static final double armMotorKA = 0.0;
    public static final double armMotorKP = 45.0;
    public static final double armMotorKI = 0.0;
    public static final double armMotorKD = 0.0;
    public static final double MMagicCruiseVelocity = 40;
    public static final double MMagicAcceleration = 80;
    public static final double MMagicJerk = 1200;
    public static final double MMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    public static final double MMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
    public static final double peakForwardVoltage = 8.0; // Peak output of 8 volts
    public static final double peakReverseVoltage = -8.0; // Peak output of 8 volts

    public static final double kArmPositionMax = 0.437;
    public static final double kArmPositionMin = 0.089;

    public static final double kTargetArmHigh = 0.437;
    public static final double kTargetArmLow = 0.089;
  }

  public static final class WristConstants {
    public static final String CANBUS = "rio";
    public static final int WRISTMOTOR_ID = 33;
    public static final int WRISTENCODER_ID = 34;

    public static final InvertedValue kWristInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kWristNeutralMode = NeutralModeValue.Brake;
    public static final SensorDirectionValue kWristEncoderDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final double kWristEncoderOffset = 0.169;

    public static final double kWristChainRatio = 1.0; // 1:1
    public static final double kWristGearboxRatio = 45.0; // 1:45
    public static final double kWristGearRatio =
        kWristChainRatio * kWristGearboxRatio; // chain ratio * Gearbox ratio

    public static final double wristMotorKG = 0.0;
    public static final double wristMotorKS = 0.0;
    public static final double wristMotorKV = 0.0;
    public static final double wristMotorKA = 0.0;
    public static final double wristMotorKP = 70.0;
    public static final double wristMotorKI = 0.0;
    public static final double wristMotorKD = 0.0;
    public static final double MMagicCruiseVelocity = 20;
    public static final double MMagicAcceleration = 40;
    public static final double MMagicJerk = 1200;
    public static final double MMagicExpo_kV = 0.13; // kV is around 0.12 V/rps
    public static final double MMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    public static final double kWristPositionMax = 0.25;
    public static final double kWristPositionMin = -0.3;

    public static final double kTargetWristHigh = 0.25;
    public static final double kTargetWristLow = -0.3;

    public static final double peakForwardVoltage = 8.0; // Peak output of 8 volts
    public static final double peakReverseVoltage = -8.0; // Peak output of 8 volts
  }

  public static final class IntakeConstants {
    public static final String CANBUS = "rio";
    public static final int LINTAKEMOTOR_ID = 35;
    public static final int RINTAKEMOTOR_ID = 36;
    public static final int RANGESENSOR_ID = 37;

    public static final InvertedValue kLIntakeInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kRIntakeInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kIntakeNeutralMode = NeutralModeValue.Coast;
    public static final double peakForwardVoltage = 10.0; // Peak output of 8 volts
    public static final double peakReverseVoltage = -10.0; // Peak output of 8 volts
    public static final double peakForwardTorqueCurrent = 40.0; // Peak output of 40 amps
    public static final double peakReverseTorqueCurrent = -40.0; // Peak output of 40 amps

    public static final double kIntakeChainRatio = 24.0 / 10.0; // 24:10
    public static final double kIntakeGearboxRatio = 1.0; // 1:1
    public static final double kIntakeGearRatio = kIntakeChainRatio * kIntakeGearboxRatio;

    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    public static final double intakeMotorTorqueKS = 0.0; // Static feedforward gain
    public static final double intakeMotorTorqueKP = 8.0; // error of 1 rps results in 8 amps output
    public static final double intakeMotorTorqueKI = 0.2; // error of 1 rps incr by 0.2 amps per sec
    public static final double intakeMotorTorqueKD =
        0.001; // A change of 1000 rps^2 incr 1 amp output

    public static final double MMagicCruiseVelocity = 40;
    public static final double MMagicAcceleration = 80;
    public static final double MMagicJerk = 800;

    public static final double intakeVelocity = -3.0;
    public static final double outtakeVelocityL = 3.0;
    public static final double outtakeVelocityR = 6.0;

    public static final Distance rangeThreshold = Inches.of(5.0);
    public static final double kRangeFOVCenterX = 0;
    public static final double kRangeFOVCenterY = 0;
    public static final double kRangeFOVRangeX = 27;
    public static final double kRangeFOVRangeY = 27;
    public static final double kProxThreshold = 0.1;
    public static final double kProxHysteresis = 0.01;
    public static final double kMinSigStrength = 2500;
  }

  public static final class ClimberConstants {
    public static final String CANBUS = "CANFD";
    public static final int CLIMBMOTOR_ID = 41;
    public static final int CLIMBENCODER_ID = 0; // Thrubore plugged into DIO 0
    public static final int CLAMPSERVO_ID = 0; // Rachet servo plugged into PWM 0
    public static final int ROTATEMOTOR_ID = 43;

    public static final InvertedValue kClimberInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kClimberNeutralMode = NeutralModeValue.Brake;
    public static final SensorDirectionValue kClimberEncoderDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    public static final double kClimberChainRatio = 40.0 / 10.0;
    public static final double kClimberGearboxRatio = 100.0; // 1:100
    public static final double kClimberGearRatio =
        kClimberChainRatio * kClimberGearboxRatio; // chain ratio * Gearbox ratio

    public static final double peakForwardVoltage = 10.0; // Peak output of 10 volts
    public static final double peakReverseVoltage = -10.0; // Peak output of 10 volts
    public static final double peakForwardTorqueCurrent = 80.0; // Peak output of 80 amps
    public static final double peakReverseTorqueCurrent = -80.0; // Peak output of 80 amps

    public static final double climbMotorKG = 0.0;
    public static final double climbMotorKS = 0.0;
    public static final double climbMotorKV = 0.0;
    public static final double climbMotorKA = 0.0;
    public static final double climbMotorKP = 4.0;
    public static final double climbMotorKI = 0.0;
    public static final double climbMotorKD = 0.0;

    public static final double climbMotorTorqueKP =
        60.0; // An error of 1 rot. results in 60 A output
    public static final double climbMotorTorqueKI = 0.0; // No output for integrated error
    public static final double climbMotorTorqueKD =
        6.0; // A velocity of 1 rps results in 6 A output

    public static final double kUnclampedPosition = 0.23;
    public static final double kClampedPosition = 0.5;

    public static final double kClimberPositionMin = 128.2; // 0.318;
    public static final double kClimberPositionMax = 215.0; // 0.503;

    public static final double kTargetClimberUp = 128.2;
    public static final double kTargetClimberDown = 215.0;

    public static final double kClimberSpeed = 0.2;
    public static final double kClimberSpeed2 = 0.5;
    public static final double kRotateSpeed = 1.0;
  }
}
