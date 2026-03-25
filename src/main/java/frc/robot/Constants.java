package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public final class Constants {
  public static final class GameManager {
    public static double ShiftAuto = 20;
    public static double ShiftTransistion = 140;
    public static double ShiftOne = 130;
    public static double ShiftTwo = 105;
    public static double ShiftThree = 80;
    public static double ShiftFour = 55;
    public static double ShiftEndGame = 30;
  }

  public static final class TurretSubsystem {

    // duty cycles
    public static double Turret_PeakForwardDutyCycle = 0.5;
    public static double Turret_PeakReverseDutyCycle = -0.5;

    public static double Turret_Slot0_kP = 0.35;
    public static double Turret_Slot0_kI = 0;
    public static double Turret_Slot0_kD = 0.003;

    public static boolean Turret_StatorCurrentLimitEnable = false;
    public static double Turret_StatorCurrentLimit = 0;
    public static boolean Turret_SupplyCurrentLimitEnable = true;
    public static double Turret_SupplyCurrentLimit = 70;
    public static double Turret_SupplyCurrentLowerLimit = 40;
    public static double Turret_SupplyCurrentLowerTime = 1;
    public static boolean Turret_FowardSoftLimitEnable = true;
    public static boolean Turret_ReverseSoftLimitEnable = true;

    // gear ratio
    public static double Turret_SensorToMechanismRatio = 0.07608695652;

    // Voltage
    public static double Turret_PeakForwardVoltage = 3.9;
    public static double Turret_PeakReverseVoltage = -3.9;

    public static double Turret_MotionMagicExpo_kV = 0;
    // Torque Current
    public static double Turret_PeakForwardTorqueCurrent = 0;
    public static double Turret_PeakReverseTorqueCurrent = 0;
    // soft limits
    public static double Turret_FowardSoftLimit = 315;
    public static double Turret_ReverseSoftLimit = -20;

  }

  public static final class OutOfBumperIntakeSubsystem {

    public static double OutBumperPivot_PeakForwardDutyCycle = 0.2;
    public static double OutBumperPivot_PeakReverseDutyCycle = -0.05;

    public static double OutBumperPivot_Slot0_kP = 8;
    public static double OutBumperPivot_Slot0_kI = 0;
    public static double OutBumperPivot_Slot0_kD = 0.7;

    public static boolean OutBumperPivot_SupplyCurrentLimitEnable = true;
    public static double OutBumperPivot_SupplyCurrentLimit = 2;
    public static boolean OutBumperPivot_FowardSoftLimitEnable = true;
    public static double OutBumperPivot_ForwardSoftLimitThreshold = 0.26;
    public static boolean OutBumperPivot_ReverseSoftLimitEnable = true;
    public static double OutBumperPivot_ReverseSoftLimitThreshold = 0;
    public static double OutBumperPivot_ForwardVoltage = 2;
    public static double OutBumperPivot_ReverseVoltage = -1.1;
  }

  public static final class IntakeSubsystem {

    public static double ThroughBumperIntake_PeakForwardDutyCycle = 1;
    public static double ThroughBumperIntake_PeakReverseDutyCycle = -1;

    public static double ThroughBumperIntake_Slot0_kP = 0;
    public static double ThroughBumperIntake_Slot0_kI = 0;
    public static double ThroughBumperIntake_Slot0_kD = 0;

    public static boolean ThroughBumperIntake_StatorCurrentLimitEnable = false;
    public static double ThroughBumperIntake_CurrentLimit = 0;
    public static boolean ThroughBumperIntake_SupplyCurrentLimitEnable = true;
    public static double ThroughBumperIntake_SupplyCurrentLimit = 40;

    // Voltage
    public static double ThroughBumperIntake_PeakForwardVoltage = 7;
    public static double ThroughBumperIntake_PeakReverseVoltage = -7;
    // Differential Constants
    public static double ThroughBumperIntake_PeakDifferentialDutyCycle = 0;
    public static double ThroughBumperIntake_PeakDifferentialTorqueCurrent = 0;
    public static double ThroughBumperIntake_PeakDifferentialVoltage = 0;
    // Motion Magic
    public static double ThroughBumperIntake_MotionMagicCruiseVelocity = 0;
    public static double ThroughBumperIntake_MotionMagicAcceleration = 0;
    public static double ThroughBumperIntake_MotionMagicExpo_kA = 0;

    public static double ThroughBumperIntake_MotionMagicExpo_kV = 0;

    // Torque Current
    public static double ThroughBumperIntake_PeakForwardTorqueCurrent = 0;
    public static double ThroughBumperIntake_PeakReverseTorqueCurrent = 0;

    // intake velocities
    public static double ThroughBumperIntake_FrontSpeed = 0;

    public static double ThroughBumperIntake_BackSpeed = 0;

  }

  public static final class HopperSubsystem {

    public static double hopper_PeakForwardDutyCycle = 1;
    public static double hopper_PeakReverseDutyCycle = -1;

    public static double hopper_Slot0_kP = 0;
    public static double hopper_Slot0_kI = 0;
    public static double hopper_Slot0_kD = 0;

    public static boolean hopper_StatorCurrentLimitEnable = false;
    public static double hopper_CurrentLimit = 0;
    public static boolean hopper_SupplyCurrentLimitEnable = true;
    public static double hopper_SupplyCurrentLimit = 40;

    // Voltage
    public static double hopper_PeakForwardVoltage = 7;
    public static double hopper_PeakReverseVoltage = -7;
    // Differential Constants
    public static double hopper_PeakDifferentialDutyCycle = 0;
    public static double hopper_PeakDifferentialTorqueCurrent = 0;
    public static double hopper_PeakDifferentialVoltage = 0;
    // Motion Magic
    public static double hopper_MotionMagicCruiseVelocity = 0;
    public static double hopper_MotionMagicAcceleration = 0;
    public static double hopper_MotionMagicExpo_kA = 0;

    public static double hopper_MotionMagicExpo_kV = 0;

    // Torque Current
    public static double hopper_PeakForwardTorqueCurrent = 0;
    public static double hopper_PeakReverseTorqueCurrent = 0;

    // intake velocities
    public static double hopper_FrontSpeed = 0;

    public static double hopper_BackSpeed = 0;

  }

  public static final class IndexerSubsystem {

    // PIDs for the index motor
    public static double index_PeakForwardDutyCycle = 1;
    public static double index_PeakReverseDutyCycle = -1;

    public static double index_Slot0_kP = 0;
    public static double index_Slot0_kI = 0;
    public static double index_Slot0_kD = 0;

    public static boolean index_StatorCurrentLimitEnable = false;
    public static double index_CurrentLimit = 0;
    public static boolean index_SupplyCurrentLimitEnable = true;
    public static double index_SupplyCurrentLimit = 40;

    // Voltage
    public static double index_PeakForwardVoltage = 7;
    public static double index_PeakReverseVoltage = -7;
    // Differential Constants
    public static double index_PeakDifferentialDutyCycle = 0;
    public static double index_PeakDifferentialTorqueCurrent = 0;
    public static double index_PeakDifferentialVoltage = 0;
    // Motion Magic
    public static double index_MotionMagicCruiseVelocity = 0;
    public static double index_MotionMagicAcceleration = 0;
    public static double index_MotionMagicExpo_kA = 0;

    public static double index_MotionMagicExpo_kV = 0;

    // Torque Current
    public static double index_PeakForwardTorqueCurrent = 0;
    public static double index_PeakReverseTorqueCurrent = 0;

    // intake velocities
    public static double index_FrontSpeed = 0;

    public static double index_BackSpeed = 0;

    //PIDs for the belt motor
    public static double belt_PeakForwardDutyCycle = 1;
    public static double belt_PeakReverseDutyCycle = -1;

    public static double belt_Slot0_kP = 0;
    public static double belt_Slot0_kI = 0;
    public static double belt_Slot0_kD = 0;

    public static boolean belt_StatorCurrentLimitEnable = false;
    public static double belt_CurrentLimit = 0;
    public static boolean belt_SupplyCurrentLimitEnable = true;
    public static double belt_SupplyCurrentLimit = 40;

    // Voltage
    public static double belt_PeakForwardVoltage = 7;
    public static double belt_PeakReverseVoltage = -7;
    // Differential Constants
    public static double belt_PeakDifferentialDutyCycle = 0;
    public static double belt_PeakDifferentialTorqueCurrent = 0;
    public static double belt_PeakDifferentialVoltage = 0;
    // Motion Magic
    public static double belt_MotionMagicCruiseVelocity = 0;
    public static double belt_MotionMagicAcceleration = 0;
    public static double belt_MotionMagicExpo_kA = 0;

    public static double belt_MotionMagicExpo_kV = 0;

    // Torque Current
    public static double belt_PeakForwardTorqueCurrent = 0;
    public static double belt_PeakReverseTorqueCurrent = 0;

    // intake velocities
    public static double belt_FrontSpeed = 0;

    public static double belt_BackSpeed = 0;



    //PIDs for the shooterIntake motor
    public static double shooterIntake_PeakForwardDutyCycle = 1;
    public static double shooterIntake_PeakReverseDutyCycle = -1;

    public static double shooterIntake_Slot0_kP = 0;
    public static double shooterIntake_Slot0_kI = 0;
    public static double shooterIntake_Slot0_kD = 0;

    public static boolean shooterIntake_StatorCurrentLimitEnable = false;
    public static double shooterIntake_CurrentLimit = 0;
    public static boolean shooterIntake_SupplyCurrentLimitEnable = true;
    public static double shooterIntake_SupplyCurrentLimit = 40;

    // Voltage
    public static double shooterIntake_PeakForwardVoltage = 7;
    public static double shooterIntake_PeakReverseVoltage = -7;
    // Differential Constants
    public static double shooterIntake_PeakDifferentialDutyCycle = 0;
    public static double shooterIntake_PeakDifferentialTorqueCurrent = 0;
    public static double shooterIntake_PeakDifferentialVoltage = 0;
    // Motion Magic
    public static double shooterIntake_MotionMagicCruiseVelocity = 0;
    public static double shooterIntake_MotionMagicAcceleration = 0;
    public static double shooterIntake_MotionMagicExpo_kA = 0;

    public static double shooterIntake_MotionMagicExpo_kV = 0;

    // Torque Current
    public static double shooterIntake_PeakForwardTorqueCurrent = 0;
    public static double shooterIntake_PeakReverseTorqueCurrent = 0;

    // intake velocities
    public static double shooterIntake_FrontSpeed = 0;

    public static double shooterIntake_BackSpeed = 0;

  }

  public static final class ShooterSubsystem {

    public static double Shooter_PeakForwardDutyCycle = 1;
    public static double Shooter_PeakReverseDutyCycle = -1;

    public static double Shooter_Slot0_kP = 0.2;
    public static double Shooter_Slot0_kI = 0;
    public static double Shooter_Slot0_kD = 0;
    public static double Shooter_Slot0_kV = 0.1195;
    public static double Shooter_Slot0_kS = 0.27;

    public static boolean Shooter_StatorCurrentLimitEnable = false;
    public static double Shooter_CurrentLimit = 10;
    public static boolean Shooter_SupplyCurrentLimitEnable = true;
    public static double Shooter_SupplyCurrentLimit = 80;
    public static double Shooter_SupplyCurrentLowerLimit = 0;
    public static double Shooter_SupplyCurrentLowerTime = 0;

    // Voltage
    public static double Shooter_PeakForwardVoltage = 16;
    public static double Shooter_PeakReverseVoltage = -16;
    // Differential Constants
    public static double Shooter_PeakDifferentialDutyCycle = 0;
    public static double Shooter_PeakDifferentialTorqueCurrent = 0;
    public static double Shooter_PeakDifferentialVoltage = 0;
    // Motion Magic
    public static double Shooter_MotionMagicCruiseVelocity = 0;
    public static double Shooter_MotionMagicAcceleration = 0;
    public static double Shooter_MotionMagicExpo_kA = 0;

    public static double Shooter_MotionMagicExpo_kV = 0;
    // Torque Current
    public static double Shooter_PeakForwardTorqueCurrent = 800;
    public static double Shooter_PeakReverseTorqueCurrent = -800;

    public static double Hood_PeakForwardDutyCycle = 0.5;
    public static double Hood_PeakReverseDutyCycle = -0.5;

    public static double Hood_Slot0_kP = 6;
    public static double Hood_Slot0_kI = 0;
    public static double Hood_Slot0_kD = 0;

    public static boolean Hood_StatorCurrentLimitEnable = false;
    public static double Hood_CurrentLimit = 10;
    public static boolean Hood_SupplyCurrentLimitEnable = true;
    public static double Hood_SupplyCurrentLimit = 15;

    // Voltage
    public static double Hood_PeakForwardVoltage = 4;
    public static double Hood_PeakReverseVoltage = -4;
    // Differential Constants
    public static double Hood_PeakDifferentialDutyCycle = 0;

    // Motion Magic
    public static double Hood_MotionMagicCruiseVelocity = 0;
    public static double Hood_MotionMagicAcceleration = 0;

  }

  public static final class AprilTagPositions {
    // setup for welded field map
    public static AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}