// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//constants
import frc.robot.Constants;

// TODO TEST MOTION MAGIC THEN DELETE TRAPEZOIDAL POSITIONING
/** Add your docs here. */
public class ClimberSubsystem extends SubsystemBase {
    public final TalonFX climberMotorA = new TalonFX(11);
    public final TalonFX climberMotorB = new TalonFX(12);
    public final CANcoder climberCANcoder = new CANcoder(13);

    private DigitalInput bottomStop = new DigitalInput(1);
    private MotionMagicVoltage m_request;

    public PIDController climberPID = new PIDController(1.1, 0, 0.13);

    // variables
    private double climberPosition;

    // like every value is stripped directly from the 2025 elevator... SHH don't
    // tell anyone!!!

    public ClimberSubsystem() {

        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();

        // stuff for feedforward
        climberConfigs.Slot0.kS = Constants.ClimberSubsystem.Climber_kS; // volts
        climberConfigs.Slot0.kG = Constants.ClimberSubsystem.Climber_kG; // volts
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
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ClimberSubsystem.Climber_MotionMagicCruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = Constants.ClimberSubsystem.Climber_MotionMagicCruiseAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = Constants.ClimberSubsystem.Climber_MotionMagicJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        climberMotorA.getConfigurator().apply(climberConfigs);
        climberMotorB.getConfigurator().apply(climberConfigs);
        climberMotorB.setControl(new Follower(11, MotorAlignmentValue.Opposed)); // sets the motor to be reversed... I
                                                                                 // think - Brian
        m_request = new MotionMagicVoltage(0);

    }

    public boolean AtGoalPosition(double GoalPosition) {
        return MathUtil.isNear(GoalPosition, climberPosition, .5);
    }

    public boolean ClimberPast(double goalPosition) {
        return climberPosition > goalPosition;
    }

    public void MoveToPosition(double newPosition) {

        climberMotorA.setControl(m_request.withPosition(newPosition).withSlot(1));

    }

    public void ResetEncoder() {
        climberMotorA.setPosition(0);
    }

    @Override
    public void periodic() {
        climberPosition = climberMotorA.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("climber pos", climberPosition);

    }

}
