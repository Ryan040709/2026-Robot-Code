package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase { // Brian didn't know whether this should be mixed between the intake and shooter, or be its own thing.

    VelocityVoltage velocity = new VelocityVoltage(0);

    private TalonFX indexerBottom = new TalonFX(23);
    private TalonFX beltMotor = new TalonFX(24);
    private TalonFX shooterIntake = new TalonFX(24);

    public IndexerSubsystem() {

        // indexer motor config
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.PeakForwardDutyCycle = Constants.IndexerSubsystem.index_PeakForwardDutyCycle;
        indexerConfig.MotorOutput.PeakReverseDutyCycle = Constants.IndexerSubsystem.index_PeakReverseDutyCycle;
        // motor "friction" type?
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        indexerConfig.Slot0.kP = Constants.IndexerSubsystem.index_Slot0_kP;
        indexerConfig.Slot0.kI = Constants.IndexerSubsystem.index_Slot0_kI;
        indexerConfig.Slot0.kD = Constants.IndexerSubsystem.index_Slot0_kD;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.IndexerSubsystem.index_StatorCurrentLimitEnable;
        indexerConfig.CurrentLimits.StatorCurrentLimit = Constants.IndexerSubsystem.index_CurrentLimit;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.IndexerSubsystem.index_SupplyCurrentLimitEnable;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = Constants.IndexerSubsystem.index_SupplyCurrentLimit;
        // Voltage
        indexerConfig.Voltage.PeakForwardVoltage = Constants.IndexerSubsystem.index_PeakForwardVoltage;
        indexerConfig.Voltage.PeakReverseVoltage = Constants.IndexerSubsystem.index_PeakReverseVoltage;
        // Differential Constants
        indexerConfig.DifferentialConstants.PeakDifferentialDutyCycle = Constants.IndexerSubsystem.index_PeakDifferentialDutyCycle;
        indexerConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = Constants.IndexerSubsystem.index_PeakDifferentialDutyCycle;
        indexerConfig.DifferentialConstants.PeakDifferentialVoltage = Constants.IndexerSubsystem.index_PeakDifferentialVoltage;
        // Motion Magic
        indexerConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IndexerSubsystem.index_MotionMagicCruiseVelocity;
        indexerConfig.MotionMagic.MotionMagicAcceleration = Constants.IndexerSubsystem.index_MotionMagicAcceleration;
        indexerConfig.MotionMagic.MotionMagicExpo_kA = Constants.IndexerSubsystem.index_MotionMagicExpo_kA;
        indexerConfig.MotionMagic.MotionMagicExpo_kV = Constants.IndexerSubsystem.index_MotionMagicExpo_kV;
        // Torque Current
        indexerConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.IndexerSubsystem.index_PeakForwardTorqueCurrent;
        indexerConfig.TorqueCurrent.PeakReverseTorqueCurrent = Constants.IndexerSubsystem.index_PeakReverseTorqueCurrent;

        indexerBottom.getConfigurator().apply(indexerConfig);



        // belt motor config
        TalonFXConfiguration beltConfig = new TalonFXConfiguration();
        beltConfig.MotorOutput.PeakForwardDutyCycle = Constants.IndexerSubsystem.belt_PeakForwardDutyCycle;
        beltConfig.MotorOutput.PeakReverseDutyCycle = Constants.IndexerSubsystem.belt_PeakReverseDutyCycle;
        // motor "friction" type?
        beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        beltConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        beltConfig.Slot0.kP = Constants.IndexerSubsystem.belt_Slot0_kP;
        beltConfig.Slot0.kI = Constants.IndexerSubsystem.belt_Slot0_kI;
        beltConfig.Slot0.kD = Constants.IndexerSubsystem.belt_Slot0_kD;
        beltConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.IndexerSubsystem.belt_StatorCurrentLimitEnable;
        beltConfig.CurrentLimits.StatorCurrentLimit = Constants.IndexerSubsystem.belt_CurrentLimit;
        beltConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.IndexerSubsystem.belt_SupplyCurrentLimitEnable;
        beltConfig.CurrentLimits.SupplyCurrentLimit = Constants.IndexerSubsystem.belt_SupplyCurrentLimit;
        // Voltage
        beltConfig.Voltage.PeakForwardVoltage = Constants.IndexerSubsystem.belt_PeakForwardVoltage;
        beltConfig.Voltage.PeakReverseVoltage = Constants.IndexerSubsystem.belt_PeakReverseVoltage;
        // Differential Constants
        beltConfig.DifferentialConstants.PeakDifferentialDutyCycle = Constants.IndexerSubsystem.belt_PeakDifferentialDutyCycle;
        beltConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = Constants.IndexerSubsystem.belt_PeakDifferentialDutyCycle;
        beltConfig.DifferentialConstants.PeakDifferentialVoltage = Constants.IndexerSubsystem.belt_PeakDifferentialVoltage;
        // Motion Magic
        beltConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IndexerSubsystem.belt_MotionMagicCruiseVelocity;
        beltConfig.MotionMagic.MotionMagicAcceleration = Constants.IndexerSubsystem.belt_MotionMagicAcceleration;
        beltConfig.MotionMagic.MotionMagicExpo_kA = Constants.IndexerSubsystem.belt_MotionMagicExpo_kA;
        beltConfig.MotionMagic.MotionMagicExpo_kV = Constants.IndexerSubsystem.belt_MotionMagicExpo_kV;
        // Torque Current
        beltConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.IndexerSubsystem.belt_PeakForwardTorqueCurrent;
        beltConfig.TorqueCurrent.PeakReverseTorqueCurrent = Constants.IndexerSubsystem.belt_PeakReverseTorqueCurrent;

        beltMotor.getConfigurator().apply(beltConfig);



        // shooter intake motor config
        TalonFXConfiguration shooterIntakeConfig = new TalonFXConfiguration();
        shooterIntakeConfig.MotorOutput.PeakForwardDutyCycle = Constants.IndexerSubsystem.shooterIntake_PeakForwardDutyCycle;
        shooterIntakeConfig.MotorOutput.PeakReverseDutyCycle = Constants.IndexerSubsystem.shooterIntake_PeakReverseDutyCycle;
        // motor "friction" type?
        shooterIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterIntakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        shooterIntakeConfig.Slot0.kP = Constants.IndexerSubsystem.shooterIntake_Slot0_kP;
        shooterIntakeConfig.Slot0.kI = Constants.IndexerSubsystem.shooterIntake_Slot0_kI;
        shooterIntakeConfig.Slot0.kD = Constants.IndexerSubsystem.shooterIntake_Slot0_kD;
        shooterIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.IndexerSubsystem.shooterIntake_StatorCurrentLimitEnable;
        shooterIntakeConfig.CurrentLimits.StatorCurrentLimit = Constants.IndexerSubsystem.shooterIntake_CurrentLimit;
        shooterIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.IndexerSubsystem.shooterIntake_SupplyCurrentLimitEnable;
        shooterIntakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.IndexerSubsystem.shooterIntake_SupplyCurrentLimit;
        // Voltage
        shooterIntakeConfig.Voltage.PeakForwardVoltage = Constants.IndexerSubsystem.shooterIntake_PeakForwardVoltage;
        shooterIntakeConfig.Voltage.PeakReverseVoltage = Constants.IndexerSubsystem.shooterIntake_PeakReverseVoltage;
        // Differential Constants
        shooterIntakeConfig.DifferentialConstants.PeakDifferentialDutyCycle = Constants.IndexerSubsystem.shooterIntake_PeakDifferentialDutyCycle;
        shooterIntakeConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = Constants.IndexerSubsystem.shooterIntake_PeakDifferentialDutyCycle;
        shooterIntakeConfig.DifferentialConstants.PeakDifferentialVoltage = Constants.IndexerSubsystem.shooterIntake_PeakDifferentialVoltage;
        // Motion Magic
        shooterIntakeConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IndexerSubsystem.shooterIntake_MotionMagicCruiseVelocity;
        shooterIntakeConfig.MotionMagic.MotionMagicAcceleration = Constants.IndexerSubsystem.shooterIntake_MotionMagicAcceleration;
        shooterIntakeConfig.MotionMagic.MotionMagicExpo_kA = Constants.IndexerSubsystem.shooterIntake_MotionMagicExpo_kA;
        shooterIntakeConfig.MotionMagic.MotionMagicExpo_kV = Constants.IndexerSubsystem.shooterIntake_MotionMagicExpo_kV;
        // Torque Current
        shooterIntakeConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.IndexerSubsystem.shooterIntake_PeakForwardTorqueCurrent;
        shooterIntakeConfig.TorqueCurrent.PeakReverseTorqueCurrent = Constants.IndexerSubsystem.shooterIntake_PeakReverseTorqueCurrent;

        shooterIntake.getConfigurator().apply(shooterIntakeConfig);
    }

    @Override
    public void periodic() {
    }

    // uses voltage for better control, much like the original intake
    public void SetIndexSpeed(double targetPower) { 
        indexerBottom.set(targetPower);
    }

    public void SetIntakeSpeed(double targetPower) { 
        shooterIntake.set(targetPower);
    }

    // uses voltage for better control, much like the original intake
    public void SetBeltVoltage(double targetVoltage) {
        beltMotor.setVoltage(targetVoltage);
    }

}

/*NOTES
 * 
 * Hopper is pulled back and forth (positioning)
 * 
 * Indexer uses one motor, separate from hopper
 * the actual hopper "slide" uses its own motor
 * 
 * motor meant to run treadmill that pushes ball to turret
 * separate motor that controls wheels guiding balls into shooter
 * 
 */