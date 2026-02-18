package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalSource;

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

    public static double Turret_PeakForwardDutyCycle = 1;
    public static double Turret_PeakReverseDutyCycle = -1;

    public static double Turret_Slot0_kP = 1;
    public static double Turret_Slot0_kI = 0.15;
    public static double Turret_Slot0_kD = 0;

    public static boolean Turret_StatorCurrentLimitEnable = true;
    public static double Turret_StatorCurrentLimit = 100;
    public static boolean Turret_SupplyCurrentLimitEnable = true;
    public static double Turret_SupplyCurrentLimit = 100;
    public static double Turret_SupplyCurrentLowerLimit = 40;
    public static double Turret_SupplyCurrentLowerTime = -40;
    public static boolean Turret_FowardSoftLimitEnable = true;
    public static boolean Turret_ReverseSoftLimitEnable = true;

    // Voltage
    public static double Turret_PeakForwardVoltage = 16;
    public static double Turret_PeakReverseVoltage = -16;
    // Differential Constants
    public static double Turret_PeakDifferentialDutyCycle = 1;
    public static double Turret_PeakDifferentialTorqueCurrent = 800;
    public static double Turret_PeakDifferentialVoltage = 16;
    // Motion Magic
    public static double Turret_MotionMagicCruiseVelocity = 100;
    public static double Turret_MotionMagicAcceleration = 150;
    public static double Turret_MotionMagicExpo_kA = 0.10;

    public static double Turret_MotionMagicExpo_kV = 0.12;
    // Torque Current
    public static double Turret_PeakForwardTorqueCurrent = 800;
    public static double Turret_PeakReverseTorqueCurrent = -800;

  }

  public static final class OutOfBumperIntakeSubsystem {

    public static double OutBumperPivot_PeakForwardDutyCycle = 1;
    public static double OutBumperPivot_PeakReverseDutyCycle = -1;

    public static double OutBumperPivot_Slot0_kP = 1;
    public static double OutBumperPivot_Slot0_kI = 0.15;
    public static double OutBumperPivot_Slot0_kD = 0;

    public static boolean OutBumperPivot_StatorCurrentLimitEnable = true;
    public static double OutBumperPivot_CurrentLimit = 100;
    public static boolean OutBumperPivot_SupplyCurrentLimitEnable = true;
    public static double OutBumperPivot_SupplyCurrentLimit = 100;
    public static double OutBumperPivot_SupplyCurrentLowerLimit = 40;
    public static double OutBumperPivot_SupplyCurrentLowerTime = -40;
    public static boolean OutBumperPivot_FowardSoftLimitEnable = true;
    public static double OutBumperPivot_ForwardSoftLimitThreshold = 140;
    public static boolean OutBumperPivot_ReverseSoftLimitEnable = true;
    public static double OutBumperPivot_ReverseSoftLimitThreshold = -140;

    // Voltage
    public static double OutBumperPivot_PeakForwardVoltage = 16;
    public static double OutBumperPivot_PeakReverseVoltage = -16;
    // Differential Constants
    public static double OutBumperPivot_PeakDifferentialDutyCycle = 1;
    public static double OutBumperPivot_PeakDifferentialTorqueCurrent = 800;
    public static double OutBumperPivot_PeakDifferentialVoltage = 16;
    // Motion Magic
    public static double OutBumperPivot_MotionMagicCruiseVelocity = 100;
    public static double OutBumperPivot_MotionMagicAcceleration = 150;
    public static double OutBumperPivot_MotionMagicExpo_kA = 0.1;
  
    public static double OutBumperPivot_MotionMagicExpo_kV = 0.12;
    
    // Torque Current
    public static double OutBumperPivot_PeakForwardTorqueCurrent = 800;
    public static double OutBumperPivot_PeakReverseTorqueCurrent = -800;

    // intake velocities
    public static double OutBumperIntake_IntakeVelocity = 2500;
    // when the motor is up, it's at its zero position
    public static double OutBumperPivot_Up = 0;
    // when the motor moves down, its position goes up
    public static double OutBumperPivot_Down = 10;

  }

  public static final class ThroughBumperIntakeSubsystem {

    public static double ThroughBumperIntake_PeakForwardDutyCycle = 1;
    public static double ThroughBumperIntake_PeakReverseDutyCycle = -1;

    public static double ThroughBumperIntake_Slot0_kP = 1;
    public static double ThroughBumperIntake_Slot0_kI = 0.15;
    public static double ThroughBumperIntake_Slot0_kD = 0;

    public static boolean ThroughBumperIntake_StatorCurrentLimitEnable = true;
    public static double ThroughBumperIntake_CurrentLimit = 100;
    public static boolean ThroughBumperIntake_SupplyCurrentLimitEnable = true;
    public static double ThroughBumperIntake_SupplyCurrentLimit = 100;
    public static double ThroughBumperIntake_SupplyCurrentLowerLimit = 40;
    public static double ThroughBumperIntake_SupplyCurrentLowerTime = -40;
    public static boolean ThroughBumperIntake_FowardSoftLimitEnable = true;
    public static double ThroughBumperIntake_ForwardSoftLimitThreshold = 140;
    public static boolean ThroughBumperIntake_ReverseSoftLimitEnable = true;
    public static double ThroughBumperIntake_ReverseSoftLimitThreshold = -140;

    // Voltage
    public static double ThroughBumperIntake_PeakForwardVoltage = 16;
    public static double ThroughBumperIntake_PeakReverseVoltage = -16;
    // Differential Constants
    public static double ThroughBumperIntake_PeakDifferentialDutyCycle = 1;
    public static double ThroughBumperIntake_PeakDifferentialTorqueCurrent = 800;
    public static double ThroughBumperIntake_PeakDifferentialVoltage = 16;
    // Motion Magic
    public static double ThroughBumperIntake_MotionMagicCruiseVelocity = 100;
    public static double ThroughBumperIntake_MotionMagicAcceleration = 150;
    public static double ThroughBumperIntake_MotionMagicExpo_kA = 0.1;

    public static double ThroughBumperIntake_MotionMagicExpo_kV = 0.12;
    
    // Torque Current
    public static double ThroughBumperIntake_PeakForwardTorqueCurrent = 800;
    public static double ThroughBumperIntake_PeakReverseTorqueCurrent = -800;

    // intake velocities
    public static double ThroughBumperIntake_FrontSpeed = 1;

    public static double ThroughBumperIntake_BackSpeed = -1;

  }

  public static final class ClimberSubsystem {
    //TODO fix PIDs
    public static double Climber_kP = 1.1;
    public static double Climber_kI = 0;
    public static double Climber_kD = 0.13;// was .15
    // volts for static energy
    public static double Climber_kS = 0.06;
    // volts to overcome gravity
    public static double Climber_kG = 0.24;
    // volts * seconds / distance
    public static double Climber_kV = 0.1265;
    // volts * seconds^2 / distance
    public static double Climber_kA = (0);
    // motion magic stuff
    public static double Climber_MotionMagicCruiseVelocity = 60;
    public static double Climber_MotionMagicCruiseAcceleration = 200;
    public static double Climber_MotionMagicJerk = 0;

    // fix all values
    public static double Climber_SupplyCurrentLimit = 80;// 60 before speed up
    public static double Climber_ForwardPercent = 0.30;
    public static double Climber_BackwardPercent = -0.30;

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

    public static double Shooter_Slot0_kP = 1;
    public static double Shooter_Slot0_kI = 0.15;
    public static double Shooter_Slot0_kD = 0;

    public static boolean Shooter_StatorCurrentLimitEnable = true;
    public static double Shooter_CurrentLimit = 100;
    public static boolean Shooter_SupplyCurrentLimitEnable = true;
    public static double Shooter_SupplyCurrentLimit = 100;
    public static double Shooter_SupplyCurrentLowerLimit = 40;
    public static double Shooter_SupplyCurrentLowerTime = -40;

    // Voltage
    public static double Shooter_PeakForwardVoltage = 16;
    public static double Shooter_PeakReverseVoltage = -16;
    // Differential Constants
    public static double Shooter_PeakDifferentialDutyCycle = 1;
    public static double Shooter_PeakDifferentialTorqueCurrent = 800;
    public static double Shooter_PeakDifferentialVoltage = 16;
    // Motion Magic
    public static double Shooter_MotionMagicCruiseVelocity = 100;
    public static double Shooter_MotionMagicAcceleration = 150;
    public static double Shooter_MotionMagicExpo_kA = 0.1;
    
    public static double Shooter_MotionMagicExpo_kV = 0.12;
    // Torque Current
    public static double Shooter_PeakForwardTorqueCurrent = 800;
    public static double Shooter_PeakReverseTorqueCurrent = -800;

    public static double Hood_PeakForwardDutyCycle = 1;
    public static double Hood_PeakReverseDutyCycle = -1;

    public static double Hood_Slot0_kP = 1;
    public static double Hood_Slot0_kI = 0.15;
    public static double Hood_Slot0_kD = 0;

    public static boolean Hood_StatorCurrentLimitEnable = true;
    public static double Hood_CurrentLimit = 100;
    public static boolean Hood_SupplyCurrentLimitEnable = true;
    public static double Hood_SupplyCurrentLimit = 100;

    // Voltage
    public static double Hood_PeakForwardVoltage = 16;
    public static double Hood_PeakReverseVoltage = -16;
    // Differential Constants
    public static double Hood_PeakDifferentialDutyCycle = 1;

    // Motion Magic
    public static double Hood_MotionMagicCruiseVelocity = 100;
    public static double Hood_MotionMagicAcceleration = 150;

  }

  public static final class AprilTagPositions {
    // setup for welded field map
    public static AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    
    // tag 18
    public static double Tag18X = 182.11;
    public static double Tag18Y = 135.09;
    public static double Tag18Z = 44.25;
    public static double Tag18Rotation = 90;
    // tag 19
    public static double Tag19X = 205.87;
    public static double Tag19Y = 144.84;
    public static double Tag19Z = 44.25;
    // tag 20
    public static double Tag20X = 205.87;
    public static double Tag20Y = 158.84;
    public static double Tag20Z = 44.25;
    public static double Tag20Rotation = 0;
    // tag 21
    public static double Tag21X = 182.11;
    public static double Tag21Y = 182.60;
    public static double Tag21Z = 44.25;
    public static double Tag21Rotation = 90;
    // tag 22
    public static double Tag22X = 182.11;
    public static double Tag22Y = 182.60;
    public static double Tag22Z = 44.25;
    // tag 24
    public static double Tag24X = 168.11;
    public static double Tag24Y = 182.60;
    public static double Tag24Z = 44.25;
    public static double Tag24Rotation = 90;
    // tag 25
    public static double Tag25X = 158.34;
    public static double Tag25Y = 172.84;
    public static double Tag25Z = 44.25;
    public static double Tag25Rotation = 180;
    // tag 26
    public static double Tag26X = 158.34;
    public static double Tag26Y = 158.84;
    public static double Tag26Z = 44.25;
    public static double Tag26Rotation = 180;
    // tag 27
    public static double Tag27X = 168.11;
    public static double Tag27Y = 135.09;
    public static double Tag27Z = 44.25;
    public static double Tag27Rotation = 270;

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}