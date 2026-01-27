// Copyright (c) FIRST and other WPILip contributors. so dont mess with us
// Open Source Software; you can modify and/or share it under the terms of
// the WPILid BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.lang.annotation.Target;
import java.util.function.Supplier;

import org.opencv.core.Mat;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.SwerveConstants.TunerSwerveDrivetrain;
//#swervesbeswervey

//

public class Shooter extends SubsystemBase {

    private Supplier<Pose2d> poseSupplier;

    private TalonFX m_ShooterL = new TalonFX(15);
    private TalonFX m_ShooterR = new TalonFX(16);
    private PositionVoltage m_request = new PositionVoltage(0);

    private final double maxAngle = 33.73877 / 90;

    private final double NinetyDegreeRotation = 33.73877;

    public final double Hy = 4.03606;// THESE ARE IN METERS NOT INCHES
    public final double redHx = 11.98482;
    public final double blueHx = 4.62554;
    public double Rx;
    public double Ry;
    public Pose2d robotPose = new Pose2d();

    public double turretHubAngle = 0;

    public double theta = 0;
    public boolean isBlue = true;

    public boolean isToggle = true;

    // im bored in need a task to do 1/17/2026 at 2:13 PM Saturday not monday or
    // smth else now its 2:14
    // private final double ticksPerAngleRatio = NinetyDegreeRotation*(360/90);

    private final double ticksPerAngle = NinetyDegreeRotation / 90;

    public static int kPigeonId = 14;

    private final Pigeon2 m_gyro = new Pigeon2(6, "rio");

    // odometry stuff
    // could probably be moved to swerve subsystem?

    // setting the postions of our swerve modules for kinematics
    private Translation2d m_frontLeftLocation = new Translation2d(.3429, .3429);
    private Translation2d m_frontRightLocation = new Translation2d(.3429, -.3429);
    private Translation2d m_backLeftLocation = new Translation2d(-.3429, .3429);
    private Translation2d m_backRightLocation = new Translation2d(-.3429, -.3429);

    private SwerveDriveOdometry m_odometry;

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
            m_backRightLocation);

    // limelight goofy ahh stuff
    // Basic targeting data
    double tx = LimelightHelpers.getTX("limelight-turret"); // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("limelight-turret"); // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("limelight-turret"); // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV("limelight-turret"); // Do you have a valid target?

    double txnc = LimelightHelpers.getTXNC("limelight-turret"); // Horizontal offset from principal pixel/point to
                                                                // target in degrees
    double tync = LimelightHelpers.getTYNC("limelight-turret"); // Vertical offset from principal pixel/point to target
                                                                // in degrees

    // VelocityVoltage Velocity = new
    // VelocityVoltage(0,0,false,0,0,false,false,false);

    VelocityVoltage velocity = new VelocityVoltage(0);

    InvertedValue Invert = InvertedValue.Clockwise_Positive;
    NeutralModeValue Coast = NeutralModeValue.Coast;
    NeutralModeValue Break = NeutralModeValue.Brake;

    public Shooter(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        // pid things
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = 1;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -1;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Slot0.kP = 1;
        motorConfig.Slot0.kI = 0.15;
        motorConfig.Slot0.kD = 0;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 100;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 100;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = -40;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 140 *
        // (ticksPerAngle);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -140 *
        // (ticksPerAngle);

        // Voltage things
        motorConfig.Voltage.PeakForwardVoltage = 16;
        motorConfig.Voltage.PeakReverseVoltage = -16;
        // Differential Constants and things like that
        motorConfig.DifferentialConstants.PeakDifferentialDutyCycle = 1;
        motorConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = 800;
        motorConfig.DifferentialConstants.PeakDifferentialVoltage = 16;
        // Motion Magic things
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        motorConfig.MotionMagic.MotionMagicAcceleration = 150;
        motorConfig.MotionMagic.MotionMagicExpo_kA = 0.10000000149011612;
        motorConfig.MotionMagic.MotionMagicExpo_kV = 0.11999999731779099;
        // Torque Current things
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
        // Set shooter motor settings
        motorConfig.MotorOutput.Inverted = Invert;
        motorConfig.MotorOutput.PeakForwardDutyCycle = .5;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -.5;
        motorConfig.MotorOutput.NeutralMode = Coast;

        m_ShooterL.getConfigurator().apply(motorConfig);
        m_ShooterR.getConfigurator().apply(motorConfig);

    }

    public void RuntoRPMs(double targetSpeed) {
        m_ShooterL.setControl(velocity.withVelocity(DistancetoRpms(calculateDistanceToHub())));
        m_ShooterR.setControl(velocity.withVelocity(-DistancetoRpms(calculateDistanceToHub())));
    }

    @Override
    public void periodic() {

    }

    public Pose2d UpdateRobotPose2d() {
        return poseSupplier.get();
    }

    public double distanceToHub() {

        if (isBlue) {
            double diffY = (Hy - Ry);
            double diffX = (blueHx - Rx);
            return Math.hypot(diffX, diffY);
        } else {
            double diffY = (Hy - Ry);
            double diffX = (redHx - Rx);
            return Math.hypot(diffX, diffY);
        }

    }

    public double DistancetoRpms(double distanceInMeters) {

        double targetRPM = (1 / 2) * (calculateDistanceToHub()) + 10; 
        //y=mx+b where "y" is the target RPM and "x" is the distance between the robot and target

        return targetRPM;
    }

    // red hub is 182.1" from the driver stations and 158.9 from side wall.
    // full field is 651.22"

    // run to golden angle
    // basic stuff, probably could probably be probably made better probably so
    // probably yeah probably uhh probably

    public void zeroGyro() {
        m_gyro.setYaw(0);
    }

    public double calculateDistanceToHub() {

        double tX = isBlue ? blueHx : redHx;
        double DistanceToTarget = Math.sqrt((Math.pow((Rx - tX), 2) + Math.pow((Ry - Hy), 2))); // (turretHubAngle-theta);

        SmartDashboard.putNumber("turretHubAngle", turretHubAngle);
        SmartDashboard.putNumber("Distance To Target", DistanceToTarget);
        return DistanceToTarget;

    }

    public double calculateRPMs() {

        return 0;

    }
}

//
//
//