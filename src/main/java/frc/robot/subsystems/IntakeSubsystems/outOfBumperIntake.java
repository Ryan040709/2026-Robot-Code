package frc.robot.subsystems.IntakeSubsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

public class outOfBumperIntake extends SubsystemBase {

    VelocityVoltage velocity = new VelocityVoltage(0);

    private TalonFX pivotMotor1 = new TalonFX(10);
    private TalonFX pivotMotor2 = new TalonFX(26);
    private TalonFX intake = new TalonFX(27);

    HopperSubsystem hopperSubsystem = new HopperSubsystem();

    public outOfBumperIntake() {

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.PeakForwardDutyCycle = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_PeakForwardDutyCycle;
        pivotConfig.MotorOutput.PeakReverseDutyCycle = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_PeakReverseDutyCycle;
        // motor "friction" type?
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // regulars
        pivotConfig.Slot0.kP = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_Slot0_kP;
        pivotConfig.Slot0.kI = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_Slot0_kI;
        pivotConfig.Slot0.kD = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_Slot0_kD;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_StatorCurrentLimitEnable;
        pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_CurrentLimit;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_SupplyCurrentLimitEnable;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_SupplyCurrentLimit;
        pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_SupplyCurrentLowerLimit;
        pivotConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_SupplyCurrentLowerTime;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_FowardSoftLimitEnable;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 140;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_ReverseSoftLimitEnable;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -140;
        // Voltage
        pivotConfig.Voltage.PeakForwardVoltage = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_PeakForwardVoltage;
        pivotConfig.Voltage.PeakReverseVoltage = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_PeakReverseVoltage;
        // Differential Constants
        pivotConfig.DifferentialConstants.PeakDifferentialDutyCycle = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_PeakDifferentialDutyCycle;
        pivotConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_PeakDifferentialDutyCycle;
        pivotConfig.DifferentialConstants.PeakDifferentialVoltage = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_PeakDifferentialVoltage;
        // Motion Magic
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_MotionMagicCruiseVelocity;
        pivotConfig.MotionMagic.MotionMagicAcceleration = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_MotionMagicAcceleration;
        pivotConfig.MotionMagic.MotionMagicExpo_kA = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_MotionMagicExpo_kA;
        pivotConfig.MotionMagic.MotionMagicExpo_kV = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_MotionMagicExpo_kV;
        // Torque Current
        pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_PeakForwardTorqueCurrent;
        pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_PeakReverseTorqueCurrent;

        pivotMotor1.getConfigurator().apply(pivotConfig);
        pivotMotor2.getConfigurator().apply(pivotConfig);
    }

    public void PivotIntake(double targetPosition) {
        pivotMotor1.setControl(new PositionVoltage(targetPosition)); // TODO CHANGE VALUE TO ACTUAL PIVOT POSITION!!!!
        pivotMotor2.setControl(new PositionVoltage(targetPosition)); // TODO CHANGE VALUE TO ACTUAL PIVOT POSITION!!!!
    }

    public void IntakeSpeed(double targetSpeed) {
        intake.setControl(velocity.withVelocity(targetSpeed));
    }

    // my gosh it's so empty... it's UGLY!!!

}