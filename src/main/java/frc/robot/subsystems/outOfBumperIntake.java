package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class outOfBumperIntake extends SubsystemBase {

    VelocityVoltage velocity = new VelocityVoltage(0);

    private TalonFX leftPivotMotor1 = new TalonFX(27);
    private TalonFX rightPivotMotor2 = new TalonFX(26);

    private PositionVoltage m_request = new PositionVoltage(0);

    private CANcoder canCoder = new CANcoder(7);
    
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
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_SupplyCurrentLimitEnable;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_SupplyCurrentLimit;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_FowardSoftLimitEnable;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.23;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_ReverseSoftLimitEnable;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        pivotConfig.Voltage.PeakForwardVoltage = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_ForwardVoltage;
        pivotConfig.Voltage.PeakReverseVoltage = Constants.OutOfBumperIntakeSubsystem.OutBumperPivot_ReverseVoltage;

        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        pivotConfig.Feedback.FeedbackRemoteSensorID = 7;

        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        leftPivotMotor1.getConfigurator().apply(pivotConfig);
        rightPivotMotor2.getConfigurator().apply(pivotConfig);

        CANcoderConfiguration canConfig = new CANcoderConfiguration();

        canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canConfig.MagnetSensor.MagnetOffset = -0.722900390625; //-0.405517578125;
        canConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0; //-0.405517578125;

        canCoder.getConfigurator().apply(canConfig);

        leftPivotMotor1.setControl(new Follower(26, MotorAlignmentValue.Opposed));
        rightPivotMotor2.setPosition(canCoder.getAbsolutePosition().getValueAsDouble());
    }

    public void PivotIntake(double targetPosition) {
        rightPivotMotor2.setControl(m_request.withPosition(targetPosition)); // TODO CHANGE VALUE TO ACTUAL PIVOT POSITION!!!!
    }
    public boolean AtPosition(double GoalPosition){
        return true; //MathUtil.isNear(GoalPosition, rightPivotMotor2.getPosition().getValueAsDouble(), .1 );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake pivot position", leftPivotMotor1.getPosition().getValueAsDouble());
    }

}