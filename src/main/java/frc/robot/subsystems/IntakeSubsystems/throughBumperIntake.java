package frc.robot.subsystems.IntakeSubsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

public class throughBumperIntake extends SubsystemBase {

    private TalonFX intakeMotor1 = new TalonFX(10);
    private TalonFX intakeMotor2 = new TalonFX(10);

    HopperSubsystem hopperSubsystem = new HopperSubsystem();

        public throughBumperIntake() {

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.PeakForwardDutyCycle = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_PeakForwardDutyCycle;
        intakeConfig.MotorOutput.PeakReverseDutyCycle = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_PeakReverseDutyCycle;
        // motor "friction" type?
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // regulars
        intakeConfig.Slot0.kP = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_Slot0_kP;
        intakeConfig.Slot0.kI = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_Slot0_kI;
        intakeConfig.Slot0.kD = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_Slot0_kD;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_StatorCurrentLimitEnable;
        intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_CurrentLimit;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_SupplyCurrentLimitEnable;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_SupplyCurrentLimit;
        intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_SupplyCurrentLowerLimit;
        intakeConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_SupplyCurrentLowerTime;
        intakeConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_FowardSoftLimitEnable;
        intakeConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 140;
        intakeConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_ReverseSoftLimitEnable;
        intakeConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -140;
        // Voltage
        intakeConfig.Voltage.PeakForwardVoltage = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_PeakForwardVoltage;
        intakeConfig.Voltage.PeakReverseVoltage = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_PeakReverseVoltage;
        // Differential Constants
        intakeConfig.DifferentialConstants.PeakDifferentialDutyCycle = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_PeakDifferentialDutyCycle;
        intakeConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_PeakDifferentialDutyCycle;
        intakeConfig.DifferentialConstants.PeakDifferentialVoltage = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_PeakDifferentialVoltage;
        // Motion Magic
        intakeConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_MotionMagicCruiseVelocity;
        intakeConfig.MotionMagic.MotionMagicAcceleration = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_MotionMagicAcceleration;
        intakeConfig.MotionMagic.MotionMagicExpo_kA = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_MotionMagicExpo_kA;
        intakeConfig.MotionMagic.MotionMagicExpo_kV = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_MotionMagicExpo_kV;
        // Torque Current
        intakeConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_PeakForwardTorqueCurrent;
        intakeConfig.TorqueCurrent.PeakReverseTorqueCurrent = Constants.ThroughBumperIntakeSubsystem.ThroughBumperIntake_PeakReverseTorqueCurrent;

        intakeMotor1.getConfigurator().apply(intakeConfig);
        intakeMotor2.getConfigurator().apply(intakeConfig);
    }

    public void MoveintakeMotor1(double targetSpeed) {
        intakeMotor1.set(targetSpeed);
    }

    public void MoveintakeMotor2(double targetSpeed) {
        intakeMotor2.set(targetSpeed);
    }

    public Command IntakeToHopperCommand() {
        return runOnce(() -> {
            MoveintakeMotor2(-0.5);
            MoveintakeMotor1(.5);
            hopperSubsystem.MoveHopperMotor(.5);
        });
    }

    public Command IntakeToTurretCommand() {
        return runOnce(() -> {
            MoveintakeMotor2(-0.5);
            MoveintakeMotor1(.5);
            hopperSubsystem.MoveHopperMotor(.5);

        });
    }

    public Command HopperToTurretCommand() {
        return runOnce(() -> {
            MoveintakeMotor2(-0.5);
            MoveintakeMotor1(.5);
            hopperSubsystem.MoveHopperMotor(.5);

        });
    }

    public Command HopperToIntakeCommand() {
        return runOnce(() -> {
            MoveintakeMotor2(-0.5);
            MoveintakeMotor1(.5);
            hopperSubsystem.MoveHopperMotor(.5);

        });
    }

    public Command IntakeStop() {
        return runOnce(() -> {
            MoveintakeMotor2(0);
            MoveintakeMotor1(0);
            hopperSubsystem.MoveHopperMotor(0);

        });
    }

}