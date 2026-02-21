package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX shooterMotorL = new TalonFX(15);
    private TalonFX shooterMotorR = new TalonFX(16);

    private TalonFXS hood = new TalonFXS(17);
    private PositionVoltage m_request = new PositionVoltage(0);
    private VelocityVoltage m_velocity = new VelocityVoltage(0);


    public static double tof = 0.25; // time of flight also very unrealistic right now...

    private Translation2d BlueHubPosition = new Translation2d(4.62554, 4.03606);
    private Translation2d RedHubPosistion = new Translation2d(11.98482, 4.03606);

    // Basic targeting data
    // in degrees
    VelocityVoltage velocity = new VelocityVoltage(0);

    InvertedValue Invert = InvertedValue.Clockwise_Positive;
    NeutralModeValue Coast = NeutralModeValue.Coast;
    NeutralModeValue Break = NeutralModeValue.Brake;

    public ShooterSubsystem() {

        // pid
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.PeakForwardDutyCycle = Constants.ShooterSubsystem.Shooter_PeakForwardDutyCycle;
        shooterConfig.MotorOutput.PeakReverseDutyCycle = Constants.ShooterSubsystem.Shooter_PeakReverseDutyCycle;
        // motor "friction" type?
        shooterConfig.MotorOutput.NeutralMode = Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // regulars
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.ShooterSubsystem.Shooter_StatorCurrentLimitEnable;
        shooterConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterSubsystem.Shooter_CurrentLimit;

        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ShooterSubsystem.Shooter_SupplyCurrentLimitEnable;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = Constants.ShooterSubsystem.Shooter_SupplyCurrentLimit;

        // Voltage
        shooterConfig.Voltage.PeakForwardVoltage = Constants.ShooterSubsystem.Shooter_PeakForwardVoltage;
        shooterConfig.Voltage.PeakReverseVoltage = Constants.ShooterSubsystem.Shooter_PeakReverseVoltage;

        // Motion Magic
        shooterConfig.Slot0.kP = Constants.ShooterSubsystem.Shooter_Slot0_kP;
        shooterConfig.Slot0.kI = Constants.ShooterSubsystem.Shooter_Slot0_kI;
        shooterConfig.Slot0.kD = Constants.ShooterSubsystem.Shooter_Slot0_kD;
        shooterConfig.Slot0.kS = Constants.ShooterSubsystem.Shooter_Slot0_kS;
        shooterConfig.Slot0.kV = Constants.ShooterSubsystem.Shooter_Slot0_kV;
        shooterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ShooterSubsystem.Shooter_MotionMagicCruiseVelocity;
        shooterConfig.MotionMagic.MotionMagicAcceleration = Constants.ShooterSubsystem.Shooter_MotionMagicAcceleration;

        shooterMotorL.getConfigurator().apply(shooterConfig);
        shooterMotorR.getConfigurator().apply(shooterConfig);

        // hood motor PID
        TalonFXSConfiguration hoodConfig = new TalonFXSConfiguration();
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

        // Voltage
        hoodConfig.Voltage.PeakForwardVoltage = Constants.ShooterSubsystem.Hood_PeakForwardVoltage;
        hoodConfig.Voltage.PeakReverseVoltage = Constants.ShooterSubsystem.Hood_PeakReverseVoltage;

        // Motion Magic
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ShooterSubsystem.Hood_MotionMagicCruiseVelocity;
        hoodConfig.MotionMagic.MotionMagicAcceleration = Constants.ShooterSubsystem.Hood_MotionMagicAcceleration;

        hood.getConfigurator().apply(hoodConfig);

        shooterMotorL.setControl(new Follower(16, MotorAlignmentValue.Opposed));

    }

    public void zeroHood() {
        hood.setPosition(0);
    }

    public void setHoodPosition(Pose2d robotPose) {
        hood.setControl(m_request.withPosition(-0.0383332*calculateHoodPosition(robotPose)+2.52999));
    }

    public void RuntoRPMs(Pose2d robotPose) {
        shooterMotorL.setControl(m_request.withVelocity(0));
        }
    public void SetShooterRPMS(double setRPMS) {
        shooterMotorL.setControl(m_velocity.withVelocity(setRPMS));
        }

    @Override
    public void periodic() {
    }

    public double CalculateRpms(Pose2d robotpose) {
        double y = 0;
        double b = 0;
        double targetRPM = y * (calculateDistanceToHub(robotpose)) + b; // the slope is a placeholder
        // y=mx+b where "y" is the target RPM and "x" is the distance between the robot
        // and target
        // to find the slope, determine positions and rpms that we know work on certain
        // spots on the field, and create a line of best fit.

        return targetRPM;
    }

    public double CalculateTof(Pose2d robotpose) {
        double y = 0;
        double b = 0;
        tof = y * (calculateDistanceToHub(robotpose)) + b; // the slope is a placeholder
        // y=mx+b where "y" is the time of flight and "x" is the distance between the robot
        // and target
        // to find the slope, determine positions and rpms that we know work on certain
        // spots on the field, and create a line of best fit.

        return tof;
    }

    public double calculateDistanceToHub(Pose2d robotPose) { // can't we just pull the botpose from the swerve drive?
        Translation2d hubPosition;

        if (GameManager.isBlueAlliance) { // used to be: DriverStation.getAlliance().orElse(Alliance.Blue) ==
                                          // Alliance.Blue
            hubPosition = BlueHubPosition;
        } else {
            hubPosition = RedHubPosistion;
        }

        double DistanceToTarget = robotPose.getTranslation().getDistance(hubPosition);

        SmartDashboard.putNumber("Distance To Target", DistanceToTarget);
        return DistanceToTarget;

    }

    public double calculateHoodPosition(Pose2d robotPose) {
        double m = 0;
        double b = 0;
        return (m * (calculateDistanceToHub(robotPose))) + b;

    }
}