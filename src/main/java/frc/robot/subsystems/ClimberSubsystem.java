package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants;

// TODO TEST MOTION MAGIC THEN DELETE TRAPEZOIDAL POSITIONING

public class ClimberSubsystem extends SubsystemBase {
    public final TalonFX climberMotorA = new TalonFX(11);
    public final TalonFX climberMotorB = new TalonFX(12);
    public final CANcoder climberCANcoder = new CANcoder(13);

    Servo climbServo = new Servo(1); // I think this one is currently being used by the LED strip.

    private MotionMagicVoltage request;

    public PIDController climberPID = new PIDController(1.1, 0, 0.13);

    private double climberPosition;

    public ClimberSubsystem() {

        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();

        climberConfigs.Slot0.kS = Constants.ClimberSubsystem.Climber_kS;
        climberConfigs.Slot0.kG = Constants.ClimberSubsystem.Climber_kG;
        climberConfigs.Slot0.kV = Constants.ClimberSubsystem.Climber_kV; // volts * seconds / distance
        // A is potentially unneccesary
        climberConfigs.Slot0.kA = Constants.ClimberSubsystem.Climber_kA; // volts * seconds^2 / distance

        climberConfigs.Slot0.kP = Constants.ClimberSubsystem.Climber_kP;
        climberConfigs.Slot0.kI = Constants.ClimberSubsystem.Climber_kI;
        climberConfigs.Slot0.kD = Constants.ClimberSubsystem.Climber_kD;

        climberConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        climberConfigs.CurrentLimits.SupplyCurrentLimit = Constants.ClimberSubsystem.Climber_SupplyCurrentLimit;

        climberConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climberConfigs.MotorOutput.PeakForwardDutyCycle = 0.3;
        climberConfigs.MotorOutput.PeakReverseDutyCycle = -0.3;
        // set Motion Magic settings

        var motionMagicConfigs = climberConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ClimberSubsystem.Climber_MotionMagicCruiseVelocity;
        // Target cruise velocity of 80 rps Target acceleration of 160 rps/s (0.5 seconds)

        // Target jerk of 1600 rps/s/s (0.1 seconds)
        motionMagicConfigs.MotionMagicJerk = Constants.ClimberSubsystem.Climber_MotionMagicJerk;
        // sets the motor to be reversed
        climberMotorA.getConfigurator().apply(climberConfigs);
        climberMotorB.getConfigurator().apply(climberConfigs);
        climberMotorB.setControl(new Follower(11, MotorAlignmentValue.Opposed));

        request = new MotionMagicVoltage(0);

    }

    public void climberServo(double targetPosition) {
        climbServo.setPosition(targetPosition);
    }

    public boolean AtGoalPosition(double GoalPosition) {
        return MathUtil.isNear(GoalPosition, climberPosition, .5);
    }

    public void MoveToPosition(double newPosition) {
        climberMotorA.setControl(request.withPosition(5.68182*newPosition).withSlot(1));
    }

    public void ResetEncoder() {
        climberMotorA.setPosition(0);
    }

    @Override
    public void periodic() {
        climberPosition = climberMotorA.getPosition().getValueAsDouble()/5.68182;
        SmartDashboard.putNumber("climber pos", climberPosition);

    }

}
