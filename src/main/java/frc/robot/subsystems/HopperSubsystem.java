package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

    private TalonFX extensionMotor = new TalonFX(25);

    private PositionVoltage positionRequest = new PositionVoltage(0);

    public HopperSubsystem() {

        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
        extensionConfig.MotorOutput.PeakForwardDutyCycle = 1;
        extensionConfig.MotorOutput.PeakReverseDutyCycle = -1;
        // motor "friction" type?
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        extensionConfig.Slot0.kP = 0;
        extensionConfig.Slot0.kI = 0;
        extensionConfig.Slot0.kD = 0;
        extensionConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        extensionConfig.CurrentLimits.SupplyCurrentLimit = 40;
        // Voltage
        extensionConfig.Voltage.PeakForwardVoltage = 7;
        extensionConfig.Voltage.PeakReverseVoltage = -7;

        extensionMotor.getConfigurator().apply(extensionConfig);
    }

    public void setHopperPosition(double targetPosition) {
        extensionMotor.setControl(positionRequest.withPosition(targetPosition));
    }
    public boolean atPosition(double GoalPosition){
        return MathUtil.isNear(GoalPosition, extensionMotor.getPosition().getValueAsDouble(), 0.1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake pivot position", extensionMotor.getPosition().getValueAsDouble());
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