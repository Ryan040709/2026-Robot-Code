package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    VelocityVoltage velocity = new VelocityVoltage(0);

    private TalonFX intakeMotor1 = new TalonFX(23);
    private TalonFX intakeMotor2 = new TalonFX(24);
    //assuming these are hopper rollers

    public IntakeSubsystem() {

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        // motor "friction" type?
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
        // Voltage
        intakeConfig.Voltage.PeakForwardVoltage = 12;
        intakeConfig.Voltage.PeakReverseVoltage = -12;
        //motion magic
        intakeConfig.Slot0.kP = .129;
        intakeConfig.Slot0.kD = .01;
        intakeConfig.Slot0.kS = .4;
        intakeConfig.Slot0.kV = .2125;
        intakeConfig.Feedback.SensorToMechanismRatio = 1.75;

        
        intakeMotor1.getConfigurator().apply(intakeConfig);
        intakeMotor2.getConfigurator().apply(intakeConfig);
        intakeMotor2.setControl(new Follower(23, MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
    }

    private void setIntakeVoltage(double targetVoltage) {
        intakeMotor1.setVoltage(targetVoltage);
    }
    private void setIntakeVelocity(double setVelocity){
        intakeMotor1.setControl(velocity.withVelocity(setVelocity));
    }


    public void IntakeForward(){
         setIntakeVelocity(35);
    }
    public void AutoIntakeForward(){
         setIntakeVelocity(40);
    }
    public void intakeReverse(){
        setIntakeVoltage(-6);
    }
    public void intakeStop(){
        setIntakeVoltage(0);
    }
    
}