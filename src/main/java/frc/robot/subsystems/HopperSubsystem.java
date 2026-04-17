package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

    private TalonFX extensionMotor = new TalonFX(25);

    private PositionVoltage positionRequest = new PositionVoltage(0);
    double extentionPosition;

    public HopperSubsystem() {

        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
        // motor "friction" type?
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        extensionConfig.Slot0.kP = 2;
        extensionConfig.Slot0.kI = 0;
        extensionConfig.Slot0.kD = 0.2;
        extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        extensionConfig.CurrentLimits.SupplyCurrentLimit = 8;
        // Voltage
        extensionConfig.Voltage.PeakForwardVoltage = 10;
        extensionConfig.Voltage.PeakReverseVoltage = -10;
        //limits
        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 32;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        extensionConfig.MotionMagic.MotionMagicAcceleration = 200;
        extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 70;

        extensionMotor.getConfigurator().apply(extensionConfig);
        
        


    }

    public void setHopperPosition(double targetPosition) {
        extensionMotor.setControl(positionRequest.withPosition(targetPosition));
    }
    public boolean atPosition(double GoalPosition){
        return MathUtil.isNear(GoalPosition, extensionMotor.getPosition().getValueAsDouble(), 1);
    }
    public boolean PastPosition(double position){
        return extensionMotor.getPosition().getValueAsDouble() > position;
    }
    public Command zeroExtention(){
       return runOnce(()->extensionMotor.setPosition(0));
    }




    @Override
    public void periodic() {
      //  SmartDashboard.putNumber("intake pivot position", extensionMotor.getPosition().getValueAsDouble());
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