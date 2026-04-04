package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    VelocityVoltage velocity = new VelocityVoltage(0);

    private TalonFX rollMotor = new TalonFX(18);
    private TalonFX beltMotor = new TalonFX(19);
    private TalonFX rampMotor = new TalonFX(20);

    double rollerSpeed;
    double beltSpeed;
    double rampSpeed;

    public IndexerSubsystem() {

        // indexer motor config
        TalonFXConfiguration rollConfig = new TalonFXConfiguration();
        // motor "friction" type?
        rollConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        rollConfig.Slot0.kP = 0.1;
        rollConfig.Slot0.kI = 0;
        rollConfig.Slot0.kD = 0;
        rollConfig.Slot0.kS = .35;
        rollConfig.Slot0.kV = .128;
        rollConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollConfig.CurrentLimits.SupplyCurrentLimit = 40;
        // Voltage
        rollConfig.Voltage.PeakForwardVoltage = 12;
        rollConfig.Voltage.PeakReverseVoltage = -12;
       
        rollMotor.getConfigurator().apply(rollConfig);



        // belt motor config
        TalonFXConfiguration beltConfig = new TalonFXConfiguration();
        // motor "friction" type?
        beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        beltConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        beltConfig.Slot0.kP = 0;
        beltConfig.Slot0.kI = 0;
        beltConfig.Slot0.kD = 0;
        beltConfig.Slot0.kS = .6;
        beltConfig.Slot0.kV = .126;
        beltConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        beltConfig.CurrentLimits.SupplyCurrentLimit = 60;
        // Voltage
        beltConfig.Voltage.PeakForwardVoltage = 12;
        beltConfig.Voltage.PeakReverseVoltage = -12;
       
        beltMotor.getConfigurator().apply(beltConfig);



        // shooterIntake motor config
        TalonFXConfiguration rampConfig = new TalonFXConfiguration();
        // motor "friction" type?
        rampConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rampConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        rampConfig.Slot0.kP = 0.25;
        rampConfig.Slot0.kI = 0;
        rampConfig.Slot0.kD = 0;
        rampConfig.Slot0.kS = .4;
        rampConfig.Slot0.kV = .121;
        rampConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rampConfig.CurrentLimits.SupplyCurrentLimit = 40;
        // Voltage
        rampConfig.Voltage.PeakForwardVoltage = 12;
        rampConfig.Voltage.PeakReverseVoltage = -12;
       
        rampMotor.getConfigurator().apply(rampConfig);
    }

    @Override
    public void periodic() {
       beltSpeed =  SmartDashboard.getNumber("BeltSpeed", 60);
       rollerSpeed = SmartDashboard.getNumber("rollerSpeed", 25);
       rampSpeed = SmartDashboard.getNumber("rampSpeed", 37);

    }

    // uses voltage for better control, much like the original intake
    public void setRollerVelocity(double targetRPM) {
        rollMotor.setControl(velocity.withVelocity(targetRPM));
    }

    public void setRampVelocity(double targetRPM) {
        rampMotor.setControl(velocity.withVelocity(targetRPM));
    }

    // uses voltage for better control, much like the original intake
    public void setBeltVelocity(double targetRPM) {
        beltMotor.setControl(velocity.withVelocity(targetRPM));
    }

    public void setRollerVoltage(double targetVoltage) {
        rollMotor.setVoltage(targetVoltage);
    }

    public void setRampVoltage(double targetVoltage) {
        rampMotor.setVoltage(targetVoltage);
    }

    // uses voltage for better control, much like the original intake
    public void setBeltVoltage(double targetVoltage) {
        beltMotor.setVoltage(targetVoltage);
    }
    public void stopIndexer(){
        setBeltVoltage(0);
            setRollerVoltage(0);
            setRampVoltage(0);
    }

    public void setIndexSpeed(){
        setBeltVelocity(30); 
            setRollerVelocity(70);
            setRampVelocity(37);
    }

    public Command indexShooter(){
        return startEnd(()-> {
            setIndexSpeed();
        },
        ()->{
             stopIndexer();
        }
        );
    }
    public Command unjamShooter(){
        return startEnd(()-> {
            setBeltVelocity(60); //50 starting velocity
            setRollerVelocity(0);
            setRampVelocity(37);
        },
        ()->{
           stopIndexer();
        }
        );
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