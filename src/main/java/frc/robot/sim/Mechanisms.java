package frc.robot.sim;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Class to keep all the mechanism-specific objects together and out of the main example
 */
public class Mechanisms {
  double HEIGHT = 1; //Controls the height of the mech2d SmartDashboard
  double WIDTH = 1; //Controls the height of the mech2d SmartDashboard
  double FX = 0;
  double TALON = 0;
  double CAN = 0;


  Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);
  /* rotor rotor Ligaments */
  MechanismLigament2d rotorArm = mech.
                                getRoot("rotorpivotPoint", 0.5, 0.2).
                                append(new MechanismLigament2d("rotorArm", .04, 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d rotorSide1 = rotorArm.append(new MechanismLigament2d("rotorSide1", 0.038267, 120, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide2 = rotorSide1.append(new MechanismLigament2d("rotorSide2", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide3 = rotorSide2.append(new MechanismLigament2d("rotorSide3", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide4 = rotorSide3.append(new MechanismLigament2d("rotorSide4", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide5 = rotorSide4.append(new MechanismLigament2d("rotorSide5", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d rotorSide6 = rotorSide5.append(new MechanismLigament2d("rotorSide6", 0.038267, 60, 6, new Color8Bit(Color.kAliceBlue)));

  /* CANcoder ligaments */
  MechanismLigament2d ccArm = mech.
                                getRoot("CANpivotPoint", 0.5, 0.2).
                                append(new MechanismLigament2d("ccArm", .1, 0, 0, new Color8Bit(Color.kAntiqueWhite)));

  MechanismLigament2d ccSide1 = ccArm.append(new MechanismLigament2d("ccSide1", 0.076535, 112.5, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide2 = ccSide1.append(new MechanismLigament2d("ccSide2", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide3 = ccSide2.append(new MechanismLigament2d("ccSide3", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide4 = ccSide3.append(new MechanismLigament2d("ccSide4", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide5 = ccSide4.append(new MechanismLigament2d("ccSide5", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide6 = ccSide5.append(new MechanismLigament2d("ccSide6", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide7 = ccSide6.append(new MechanismLigament2d("ccSide7", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccSide8 = ccSide7.append(new MechanismLigament2d("ccSide8", 0.076535, 45, 6, new Color8Bit(Color.kAntiqueWhite)));
  MechanismLigament2d ccArmLen = ccArm.append(new MechanismLigament2d("ccArmLen", 0.3, 0, 8, new Color8Bit(Color.kBlue)));

  /* FX Mechanism Ligaments */
  MechanismLigament2d ccWrist = ccArmLen.append(new MechanismLigament2d("ccWrist", .025, 0, 6, new Color8Bit(Color.kOrange)));
  MechanismLigament2d wristSide2 = ccWrist.append(new MechanismLigament2d("wristSide2", 0.2, -90, 6, new Color8Bit(Color.kOrange)));
  MechanismLigament2d wristSide3 = wristSide2.append(new MechanismLigament2d("wristSide3", 0.05, -90, 6, new Color8Bit(Color.kOrange)));
  MechanismLigament2d wristSide4 = wristSide3.append(new MechanismLigament2d("wristSide4", 0.2, -90, 6, new Color8Bit(Color.kOrange)));
  MechanismLigament2d wristSide5 = wristSide4.append(new MechanismLigament2d("wristSide5", 0.025, -90, 6, new Color8Bit(Color.kOrange)));

  public void update(double armRotorPosition, double armEncoderPosition, double wristEncoderPosition) {
    // BaseStatusSignal.refreshAll(armRotorPosition, armEncoderPosition, wristEncoderPosition);
    rotorArm.setAngle(armRotorPosition * 360);
    ccArm.setAngle(armEncoderPosition * 360);
    ccWrist.setAngle(wristEncoderPosition * 360);
    SmartDashboard.putData("mech2d", mech); // Creates mech2d in SmartDashboard
  }
}