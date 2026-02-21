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

    public static double Turret_PeakForwardDutyCycle = 0;
    public static double Turret_PeakReverseDutyCycle = 0;

    public static double Turret_Slot0_kP = 0;
    public static double Turret_Slot0_kI = 0;
    public static double Turret_Slot0_kD = 0;

    public static boolean Turret_StatorCurrentLimitEnable = true;
    public static double Turret_StatorCurrentLimit = 0;
    public static boolean Turret_SupplyCurrentLimitEnable = true;
    public static double Turret_SupplyCurrentLimit = 0;
    public static double Turret_SupplyCurrentLowerLimit = 0;
    public static double Turret_SupplyCurrentLowerTime = 0;
    public static boolean Turret_FowardSoftLimitEnable = true;
    public static boolean Turret_ReverseSoftLimitEnable = true;

    // Voltage
    public static double Turret_PeakForwardVoltage = 0;
    public static double Turret_PeakReverseVoltage = 0;
    // Differential Constants
    public static double Turret_PeakDifferentialDutyCycle = 0;
    public static double Turret_PeakDifferentialTorqueCurrent = 0;
    public static double Turret_PeakDifferentialVoltage = 0;
    // Motion Magic
    public static double Turret_MotionMagicCruiseVelocity = 0;
    public static double Turret_MotionMagicAcceleration = 0;
    public static double Turret_MotionMagicExpo_kA = 0;

    public static double Turret_MotionMagicExpo_kV = 0;
    // Torque Current
    public static double Turret_PeakForwardTorqueCurrent = 0;
    public static double Turret_PeakReverseTorqueCurrent = 0;

  }

  public static final class OutOfBumperIntakeSubsystem {

    public static double OutBumperPivot_PeakForwardDutyCycle = 0;
    public static double OutBumperPivot_PeakReverseDutyCycle = 0;

    public static double OutBumperPivot_Slot0_kP = 0;
    public static double OutBumperPivot_Slot0_kI = 0;
    public static double OutBumperPivot_Slot0_kD = 0;

    public static boolean OutBumperPivot_StatorCurrentLimitEnable = true;
    public static double OutBumperPivot_CurrentLimit = 0;
    public static boolean OutBumperPivot_SupplyCurrentLimitEnable = true;
    public static double OutBumperPivot_SupplyCurrentLimit = 0;
    public static double OutBumperPivot_SupplyCurrentLowerLimit = 0;
    public static double OutBumperPivot_SupplyCurrentLowerTime = 0;
    public static boolean OutBumperPivot_FowardSoftLimitEnable = true;
    public static double OutBumperPivot_ForwardSoftLimitThreshold = 0;
    public static boolean OutBumperPivot_ReverseSoftLimitEnable = true;
    public static double OutBumperPivot_ReverseSoftLimitThreshold = 0;

    // Voltage
    public static double OutBumperPivot_PeakForwardVoltage = 0;
    public static double OutBumperPivot_PeakReverseVoltage = 0;
    // Differential Constants
    public static double OutBumperPivot_PeakDifferentialDutyCycle = 0;
    public static double OutBumperPivot_PeakDifferentialTorqueCurrent = 0;
    public static double OutBumperPivot_PeakDifferentialVoltage = 0;
    // Motion Magic
    public static double OutBumperPivot_MotionMagicCruiseVelocity = 0;
    public static double OutBumperPivot_MotionMagicAcceleration = 0;
    public static double OutBumperPivot_MotionMagicExpo_kA = 0;
  
    public static double OutBumperPivot_MotionMagicExpo_kV = 0;
    
    // Torque Current
    public static double OutBumperPivot_PeakForwardTorqueCurrent = 0;
    public static double OutBumperPivot_PeakReverseTorqueCurrent = 0;

    // intake velocities
    public static double OutBumperIntake_IntakeVelocity = 0;
    // when the motor is up, it's at its zero position
    public static double OutBumperPivot_Up = 0;
    // when the motor moves down, its position goes up
    public static double OutBumperPivot_Down = 0;

  }

  public static final class ThroughBumperIntakeSubsystem {

    public static double ThroughBumperIntake_PeakForwardDutyCycle = 1;
    public static double ThroughBumperIntake_PeakReverseDutyCycle = -1;

    public static double ThroughBumperIntake_Slot0_kP = 0;
    public static double ThroughBumperIntake_Slot0_kI = 0;
    public static double ThroughBumperIntake_Slot0_kD = 0;

    public static boolean ThroughBumperIntake_StatorCurrentLimitEnable = false;
    public static double ThroughBumperIntake_CurrentLimit = 0;
    public static boolean ThroughBumperIntake_SupplyCurrentLimitEnable = false;
    public static double ThroughBumperIntake_SupplyCurrentLimit = 10;
    public static double ThroughBumperIntake_SupplyCurrentLowerLimit = 0;
    public static double ThroughBumperIntake_SupplyCurrentLowerTime = 0;
    public static boolean ThroughBumperIntake_FowardSoftLimitEnable = false;
    public static double ThroughBumperIntake_ForwardSoftLimitThreshold = 0;
    public static boolean ThroughBumperIntake_ReverseSoftLimitEnable = false;
    public static double ThroughBumperIntake_ReverseSoftLimitThreshold = 0;

    // Voltage
    public static double ThroughBumperIntake_PeakForwardVoltage = 8;
    public static double ThroughBumperIntake_PeakReverseVoltage = -8;
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

  public static final class ClimberSubsystem {
    //TODO fix PIDs
    public static double Climber_kP = 0;
    public static double Climber_kI = 0;
    public static double Climber_kD = 0;// was .15
    // volts for static energy
    public static double Climber_kS = 0;
    // volts to overcome gravity
    public static double Climber_kG = 0;
    // volts * seconds / distance
    public static double Climber_kV = 0;
    // volts * seconds^2 / distance
    public static double Climber_kA = (0);
    // motion magic stuff
    public static double Climber_MotionMagicCruiseVelocity = 0;
    public static double Climber_MotionMagicCruiseAcceleration = 0;
    public static double Climber_MotionMagicJerk = 0;

    // fix all values
    public static double Climber_SupplyCurrentLimit = 0;// 60 before speed up
    public static double Climber_ForwardPercent = 0;
    public static double Climber_BackwardPercent = 0;

    // Climber Positions
    public static double Climber_First_Bar_Pos = 0;
    public static double Climber_Upper_Bar_Pos = 0;
    public static double Climber_Down_Pos = 0;
    public static double Climber_Hooks_Closed_Pos = 0;
    public static double Climber_Hooks_Opened_Pose= 0;

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
    public static boolean Shooter_SupplyCurrentLimitEnable = false;
    public static double Shooter_SupplyCurrentLimit = 10;
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

    public static double Hood_Slot0_kP = 2;
    public static double Hood_Slot0_kI = 0;
    public static double Hood_Slot0_kD = 0;

    public static boolean Hood_StatorCurrentLimitEnable = false;
    public static double Hood_CurrentLimit = 10;
    public static boolean Hood_SupplyCurrentLimitEnable = true;
    public static double Hood_SupplyCurrentLimit = 10;

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