// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class hoodSubsystem extends SubsystemBase {

    private TalonFXS hood = new TalonFXS(17);

    private PositionVoltage m_request = new PositionVoltage(0);

    public hoodSubsystem() {
        // hood motor PID
        TalonFXSConfiguration hoodConfig = new TalonFXSConfiguration();
        hoodConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hoodConfig.MotorOutput.PeakForwardDutyCycle = Constants.ShooterSubsystem.Hood_PeakForwardDutyCycle;
        hoodConfig.MotorOutput.PeakReverseDutyCycle = Constants.ShooterSubsystem.Hood_PeakReverseDutyCycle;
        // motor "friction" type?
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // regulars
        hoodConfig.Slot0.kP = Constants.ShooterSubsystem.Hood_Slot0_kP;
        hoodConfig.Slot0.kI = Constants.ShooterSubsystem.Hood_Slot0_kI;
        hoodConfig.Slot0.kD = Constants.ShooterSubsystem.Hood_Slot0_kD;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.ShooterSubsystem.Hood_StatorCurrentLimitEnable;
        hoodConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterSubsystem.Hood_CurrentLimit;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ShooterSubsystem.Hood_SupplyCurrentLimitEnable;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = Constants.ShooterSubsystem.Hood_SupplyCurrentLimit;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.72;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        // Voltage
        hoodConfig.Voltage.PeakForwardVoltage = Constants.ShooterSubsystem.Hood_PeakForwardVoltage;
        hoodConfig.Voltage.PeakReverseVoltage = Constants.ShooterSubsystem.Hood_PeakReverseVoltage;

        // Motion Magic
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ShooterSubsystem.Hood_MotionMagicCruiseVelocity;
        hoodConfig.MotionMagic.MotionMagicAcceleration = Constants.ShooterSubsystem.Hood_MotionMagicAcceleration;

        hood.getConfigurator().apply(hoodConfig);

    }

    public void zeroHood() {
        hood.setPosition(0); // goofy ahh comment
    }

    public void moveHood(double speed) {
        hood.set(speed);
    }
    public void moveHoodToPosition(double setPoint){
         hood.setControl(m_request.withPosition(setPoint));
    }

    public void setHoodPosition(double distanceToTarget, Pose2d turretPose, ChassisSpeeds chassisSpeeds,
            boolean isFeeding) {

        if (!robotIsNeartrench(turretPose, chassisSpeeds)) {
            if (isFeeding) {
                hood.setControl(m_request.withPosition(calculateHoodPosition(distanceToTarget, isFeeding)));
            } else {
                hood.setControl(m_request.withPosition(calculateHoodPosition(distanceToTarget, isFeeding)));
            }

        } else {
            hood.setControl(m_request.withPosition(0));
        }
    }

    public boolean robotIsNeartrench(Pose2d robotPose, ChassisSpeeds robotSpeed) {
        double blueTrenchOffset = 4.66;
        double redTrenchOffset = 11.936;
        double lowY = 1.3;
        double highY = 6.7;

        double trenchOffset;
        double tolerance;
        boolean nearSideOfTrench;

        tolerance = Math.abs(robotSpeed.vxMetersPerSecond) * .05;

        nearSideOfTrench = (robotPose.getY() < lowY || robotPose.getY() > highY);

        if (GameManager.isBlueAlliance) {
            trenchOffset = blueTrenchOffset;
        } else {
            trenchOffset = redTrenchOffset;
        }

        return (MathUtil.isNear(blueTrenchOffset, robotPose.getX(), 1 + tolerance)
        ||MathUtil.isNear(redTrenchOffset, robotPose.getX(), 1 + tolerance)
         && nearSideOfTrench);

    }

    public double calculateHoodPosition(double distanceToHub, boolean isFeeding) {
        double hubM = .222;// -5.555;
        double hubB = -.1111;
        double feedM = 0.218818;
        double feedB = -0.423414;
        if (isFeeding == true) {
            return MathUtil.clamp(( feedM * (distanceToHub)) + feedB, 0.5, 1.5);
        } else {
            return MathUtil.clamp((hubM * (distanceToHub)) + hubB, 0, 1.5);
        }

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("hood position", hood.getPosition().getValueAsDouble());
    }

}
