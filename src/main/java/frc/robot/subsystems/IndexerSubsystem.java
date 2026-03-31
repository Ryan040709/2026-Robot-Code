package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    VelocityVoltage velocity = new VelocityVoltage(0);

    private TalonFX indexerBottom = new TalonFX(18);
    private TalonFX beltMotor = new TalonFX(19);
    private TalonFX shooterIntake = new TalonFX(20);

    public IndexerSubsystem() {

        // indexer motor config
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.PeakForwardDutyCycle = 1;
        indexerConfig.MotorOutput.PeakReverseDutyCycle = -1;
        // motor "friction" type?
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        indexerConfig.Slot0.kP = 0;
        indexerConfig.Slot0.kI = 0;
        indexerConfig.Slot0.kD = 0;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        // Voltage
        indexerConfig.Voltage.PeakForwardVoltage = 7;
        indexerConfig.Voltage.PeakReverseVoltage = -7;
       
        indexerBottom.getConfigurator().apply(indexerConfig);



        // belt motor config
        TalonFXConfiguration beltConfig = new TalonFXConfiguration();
        beltConfig.MotorOutput.PeakForwardDutyCycle = 1;
        beltConfig.MotorOutput.PeakReverseDutyCycle = -1;
        // motor "friction" type?
        beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        beltConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        beltConfig.Slot0.kP = 0;
        beltConfig.Slot0.kI = 0;
        beltConfig.Slot0.kD = 0;
        beltConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        beltConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        beltConfig.CurrentLimits.SupplyCurrentLimit = 40;
        // Voltage
        beltConfig.Voltage.PeakForwardVoltage = 7;
        beltConfig.Voltage.PeakReverseVoltage = -7;
       
        beltMotor.getConfigurator().apply(beltConfig);



        // shooterIntake motor config
        TalonFXConfiguration shooterIntakeConfig = new TalonFXConfiguration();
        shooterIntakeConfig.MotorOutput.PeakForwardDutyCycle = 1;
        shooterIntakeConfig.MotorOutput.PeakReverseDutyCycle = -1;
        // motor "friction" type?
        shooterIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterIntakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        shooterIntakeConfig.Slot0.kP = 0;
        shooterIntakeConfig.Slot0.kI = 0;
        shooterIntakeConfig.Slot0.kD = 0;
        shooterIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        shooterIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterIntakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
        // Voltage
        shooterIntakeConfig.Voltage.PeakForwardVoltage = 7;
        shooterIntakeConfig.Voltage.PeakReverseVoltage = -7;
       
        shooterIntake.getConfigurator().apply(shooterIntakeConfig);
    }

    @Override
    public void periodic() {
    }

    // uses voltage for better control, much like the original intake
    public void setIndexVelocity(double targetRPM) {
        indexerBottom.setControl(velocity.withVelocity(targetRPM));
    }

    public void setIntakeVelocity(double targetRPM) {
        shooterIntake.setControl(velocity.withVelocity(targetRPM));
    }

    // uses voltage for better control, much like the original intake
    public void setBeltVelocity(double targetRPM) {
        beltMotor.setControl(velocity.withVelocity(targetRPM));
    }
    public void setIndexVoltage(double targetVoltage) {
        indexerBottom.setVoltage(targetVoltage);
    }

    public void setIntakeVoltage(double targetVoltage) {
        shooterIntake.setVoltage(targetVoltage);
    }

    // uses voltage for better control, much like the original intake
    public void setBeltVoltage(double targetVoltage) {
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