// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalSource;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

// here's brian's honest opinion on this if anyone cares:
// the "constants" file serves no honest purpose other than making it harder to
// read the code.
// I don't see the reason for its existance outside of adding more variables,
// and I like variables. :/

public final class Constants {
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
    public static double Turret_MotionMagicExpo_kA = 0.10000000149011612; // where did these super precise values come
                                                                          // from?
    public static double Turret_MotionMagicExpo_kV = 0.11999999731779099; // where did these super precise values come
                                                                          // from?
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
    public static double OutBumperPivot_MotionMagicExpo_kA = 0.10000000149011612; // where did these super precise
                                                                                  // values come
    // from?
    public static double OutBumperPivot_MotionMagicExpo_kV = 0.11999999731779099; // where did these super precise
                                                                                  // values come
    // from?
    // Torque Current
    public static double OutBumperPivot_PeakForwardTorqueCurrent = 800;
    public static double OutBumperPivot_PeakReverseTorqueCurrent = -800;

    // intake velocities
    public static double OutBumperIntake_IntakeVelocity = 2500;

    public static double OutBumperPivot_Up = 0; // when the motor is up, it's at its zero position

    public static double OutBumperPivot_Down = 10; // when the motor moves down, its position goes up

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
    public static double ThroughBumperIntake_MotionMagicExpo_kA = 0.10000000149011612; // where did these super precise
    // values come
    // from?
    public static double ThroughBumperIntake_MotionMagicExpo_kV = 0.11999999731779099; // where did these super precise
    // values come
    // from?
    // Torque Current
    public static double ThroughBumperIntake_PeakForwardTorqueCurrent = 800;
    public static double ThroughBumperIntake_PeakReverseTorqueCurrent = -800;

    // intake velocities
    public static double ThroughBumperIntake_IntakeVelocity = 2500;

  }

  public static final class ClimberSubsystem {
    // fix PIDs later!!!!
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
    public static double Climber_Up = 25;// 11 before
    public static double Climber_Down = 0;

  }

  public static final class ShooterSubsystem {

    // shooter stuff
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
    public static boolean Shooter_FowardSoftLimitEnable = true;
    public static double Shooter_ForwardSoftLimitThreshold = 140;
    public static boolean Shooter_ReverseSoftLimitEnable = true;
    public static double Shooter_ReverseSoftLimitThreshold = -140;

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
    public static double Shooter_MotionMagicExpo_kA = 0.10000000149011612; // where did these super precise
                                                                           // values come
    // from?
    public static double Shooter_MotionMagicExpo_kV = 0.11999999731779099; // where did these super precise
                                                                           // values come
    // from?
    // Torque Current
    public static double Shooter_PeakForwardTorqueCurrent = 800;
    public static double Shooter_PeakReverseTorqueCurrent = -800;

    // hood stuff
    public static double Hood_PeakForwardDutyCycle = 1;
    public static double Hood_PeakReverseDutyCycle = -1;

    public static double Hood_Slot0_kP = 1;
    public static double Hood_Slot0_kI = 0.15;
    public static double Hood_Slot0_kD = 0;

    public static boolean Hood_StatorCurrentLimitEnable = true;
    public static double Hood_CurrentLimit = 100;
    public static boolean Hood_SupplyCurrentLimitEnable = true;
    public static double Hood_SupplyCurrentLimit = 100;
    public static double Hood_SupplyCurrentLowerLimit = 40;
    public static double Hood_SupplyCurrentLowerTime = -40;
    public static boolean Hood_FowardSoftLimitEnable = true;
    public static double Hood_ForwardSoftLimitThreshold = 140;
    public static boolean Hood_ReverseSoftLimitEnable = true;
    public static double Hood_ReverseSoftLimitThreshold = -140;

    // Voltage
    public static double Hood_PeakForwardVoltage = 16;
    public static double Hood_PeakReverseVoltage = -16;
    // Differential Constants
    public static double Hood_PeakDifferentialDutyCycle = 1;
    public static double Hood_PeakDifferentialTorqueCurrent = 800;
    public static double Hood_PeakDifferentialVoltage = 16;
    // Motion Magic
    public static double Hood_MotionMagicCruiseVelocity = 100;
    public static double Hood_MotionMagicAcceleration = 150;
    public static double Hood_MotionMagicExpo_kA = 0.10000000149011612; // where did these super precise
                                                                        // values come
    // from?
    public static double Hood_MotionMagicExpo_kV = 0.11999999731779099; // where did these super precise
                                                                        // values come
    // from?
    // Torque Current
    public static double Hood_PeakForwardTorqueCurrent = 800;
    public static double Hood_PeakReverseTorqueCurrent = -800;

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}