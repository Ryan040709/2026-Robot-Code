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
import frc.robot.subsystems.HopperSubsystem;

public class outOfBumperIntake extends SubsystemBase {

    private TalonFX pivotMotor1 = new TalonFX(10);
    private TalonFX pivotMotor2 = new TalonFX(26);
    private TalonFX intake = new TalonFX(27);

    HopperSubsystem hopperSubsystem = new HopperSubsystem();

    public outOfBumperIntake() {
        // pid things
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = 1;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -1;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Slot0.kP = 1;
        motorConfig.Slot0.kI = 0.15;
        motorConfig.Slot0.kD = 0;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 100;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 100;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = -40;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 140;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -140;

        // Voltage things
        motorConfig.Voltage.PeakForwardVoltage = 16;
        motorConfig.Voltage.PeakReverseVoltage = -16;
        // Differential Constants and things like that
        motorConfig.DifferentialConstants.PeakDifferentialDutyCycle = 1;
        motorConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = 800;
        motorConfig.DifferentialConstants.PeakDifferentialVoltage = 16;
        // Motion Magic things
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        motorConfig.MotionMagic.MotionMagicAcceleration = 150;
        motorConfig.MotionMagic.MotionMagicExpo_kA = 0.10000000149011612;
        motorConfig.MotionMagic.MotionMagicExpo_kV = 0.11999999731779099;
        // Torque Current things
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;

        pivotMotor1.getConfigurator().apply(motorConfig);
        pivotMotor1.getConfigurator().apply(motorConfig);
    }

    public void lowerIntake() {
        pivotMotor1.setControl(new PositionVoltage(10)); // TODO CHANGE VALUE TO ACTUAL PIVOT POSITION!!!!
        pivotMotor2.setControl(new PositionVoltage(10)); // TODO CHANGE VALUE TO ACTUAL PIVOT POSITION!!!!
    }

    public void raiseIntake() {
        pivotMotor1.setControl(new PositionVoltage(0)); // TODO CHANGE VALUE TO ACTUAL PIVOT POSITION!!!!
        pivotMotor2.setControl(new PositionVoltage(0)); // TODO CHANGE VALUE TO ACTUAL PIVOT POSITION!!!!
    }

    public void runIntake() {
       intake.set(1);
    }

    public void runOuttake() {
       intake.set(-1);
    }

    public void stopIntake() {
       intake.set(0);
    }

}