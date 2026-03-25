package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {

    VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private TalonFX pivotMotor = new TalonFX(23);

    // after talking with Kellen, he said we can probably add a CANcoder to the hopper. Feel free to 
    private CANcoder canCoder = new CANcoder(7); 

    private PositionVoltage positionRequest = new PositionVoltage(0);

    public HopperSubsystem() {

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.PeakForwardDutyCycle = Constants.HopperSubsystem.hopper_PeakForwardDutyCycle;
        intakeConfig.MotorOutput.PeakReverseDutyCycle = Constants.HopperSubsystem.hopper_PeakReverseDutyCycle;
        // motor "friction" type?
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        intakeConfig.Slot0.kP = Constants.HopperSubsystem.hopper_Slot0_kP;
        intakeConfig.Slot0.kI = Constants.HopperSubsystem.hopper_Slot0_kI;
        intakeConfig.Slot0.kD = Constants.HopperSubsystem.hopper_Slot0_kD;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.HopperSubsystem.hopper_StatorCurrentLimitEnable;
        intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.HopperSubsystem.hopper_CurrentLimit;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.HopperSubsystem.hopper_SupplyCurrentLimitEnable;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.HopperSubsystem.hopper_SupplyCurrentLimit;
        // Voltage
        intakeConfig.Voltage.PeakForwardVoltage = Constants.HopperSubsystem.hopper_PeakForwardVoltage;
        intakeConfig.Voltage.PeakReverseVoltage = Constants.HopperSubsystem.hopper_PeakReverseVoltage;
        // Differential Constants
        intakeConfig.DifferentialConstants.PeakDifferentialDutyCycle = Constants.HopperSubsystem.hopper_PeakDifferentialDutyCycle;
        intakeConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = Constants.HopperSubsystem.hopper_PeakDifferentialDutyCycle;
        intakeConfig.DifferentialConstants.PeakDifferentialVoltage = Constants.HopperSubsystem.hopper_PeakDifferentialVoltage;
        // Motion Magic
        intakeConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HopperSubsystem.hopper_MotionMagicCruiseVelocity;
        intakeConfig.MotionMagic.MotionMagicAcceleration = Constants.HopperSubsystem.hopper_MotionMagicAcceleration;
        intakeConfig.MotionMagic.MotionMagicExpo_kA = Constants.HopperSubsystem.hopper_MotionMagicExpo_kA;
        intakeConfig.MotionMagic.MotionMagicExpo_kV = Constants.HopperSubsystem.hopper_MotionMagicExpo_kV;
        // Torque Current
        intakeConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.HopperSubsystem.hopper_PeakForwardTorqueCurrent;
        intakeConfig.TorqueCurrent.PeakReverseTorqueCurrent = Constants.HopperSubsystem.hopper_PeakReverseTorqueCurrent;

        pivotMotor.getConfigurator().apply(intakeConfig);

        CANcoderConfiguration canConfig = new CANcoderConfiguration();

        canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canConfig.MagnetSensor.MagnetOffset = -0.722900390625; //-0.405517578125;
        canConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0; //-0.405517578125;

        canCoder.getConfigurator().apply(canConfig);
    }

    public void pivotIntake(double targetPosition) {
        pivotMotor.setControl(positionRequest.withPosition(targetPosition));
    }
    public boolean atPosition(double GoalPosition){
        //return MathUtil.isNear(GoalPosition, pivotMotor.getPosition().getValueAsDouble(), 0.1);
        return MathUtil.isNear(GoalPosition, canCoder.getPosition().getValueAsDouble(), 0.1); // TODO: REMOVE THIS IF WE DON'T END UP USING A CANCODER!!!

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake pivot position", pivotMotor.getPosition().getValueAsDouble());
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