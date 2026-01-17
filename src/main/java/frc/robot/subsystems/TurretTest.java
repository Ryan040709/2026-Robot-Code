// Copyright (c) FIRST and other WPILib contributors. so dont mess with us
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.lang.annotation.Target;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.SwerveConstants.TunerSwerveDrivetrain;
//#swervesbeswervey

//

public class TurretTest extends SubsystemBase {
    // our motor exploded
    private TalonFX m_turret = new TalonFX(10);
    // said motor's pos, request stuff hi brian
    private PositionVoltage m_request = new PositionVoltage(0);

    private final double maxAngle = 33.73877 / 90;

    private final double NinetyDegreeRotation = 33.73877;

    public final double Hx = 4.03606;// THESE ARE IN METERS NOT INCHES
    public final double redHy = 4.62534;
    public final double blueHy = 11.89482;
    public double Rx = 0;
    public double Ry = 0;
    public double theta = 0;
    public boolean isBlue = true;

    // im bored in need a task to do 1/17/2026 at 2:13 PM Saturday not monday or
    // smth else now its 2:14
    // private final double ticksPerAngleRatio = NinetyDegreeRotation*(360/90);

    private final double ticksPerAngle = NinetyDegreeRotation / 90;

    public static int kPigeonId = 14;

    private final Pigeon2 m_gyro = new Pigeon2(6, "rio");

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

    public TurretTest() {
        // pid things
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = 0.75;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -0.75;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Slot0.kP = 1;
        motorConfig.Slot0.kI = 0.15;
        motorConfig.Slot0.kD = 0;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 100;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 100;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 35;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -35;

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

        m_turret.getConfigurator().apply(motorConfig);

    }

    public void MoveMotor(double targetSpeed) {
        if (m_turret.getPosition().getValueAsDouble() > -maxAngle
                || m_turret.getPosition().getValueAsDouble() < maxAngle) {
            m_turret.set(targetSpeed);
        } else {
            m_turret.set(0);
        }

    }

    // SmartDashboard.putNumber("Bridge Angle", bridgeTipper.getPosition());

    @Override
    public void periodic() {
        // StatusSignal<Angle> robotYaw = m_gyro.getYaw();

        // angle.in(Unit.degrees) robotYaw = m_gyro.getYaw();

        Angle gyroYaw = m_gyro.getYaw().getValue();

        double yaw = gyroYaw.in(Degrees);

        LimelightHelpers.SetRobotOrientation("limelight-turret", yaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Get the pose estimate now plz
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-turret");

        // Add it to your pose estimator right now

        // LimelightHelpers.PoseEstimate
        // well hello there
        // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5,
        // 9999999));
        // m_poseEstimator.addVisionMeasurement(
        // limelightMeasurement.pose,
        // limelightMeasurement.timestampSeconds
        // );

        // SmartDashboard.putNumber("Megatag",
        // m_turret.getPosition().getValueAsDouble());

        // golden angle math
        // golden angle math

        SmartDashboard.putNumber("tx", LimelightHelpers.getTX("limelight-turret"));
        SmartDashboard.putNumber("ty", LimelightHelpers.getTY("limelight-turret"));

        Pose3d x = LimelightHelpers.getBotPose3d("limelight-turret");

        double[] defaultPose = new double[6];
        double[] botpose = NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("botpose")
                .getDoubleArray(defaultPose);

        // Push values to SmartDashboard like these
        SmartDashboard.putNumber("Limelight X (m)", botpose[0]);
        SmartDashboard.putNumber("Limelight Y (m)", botpose[1]);
        SmartDashboard.putNumber("Limelight Z (m)", botpose[2]);
        SmartDashboard.putNumber("Limelight Roll (deg)", botpose[3]);
        SmartDashboard.putNumber("Limelight Pitch (deg)", botpose[4]);
        SmartDashboard.putNumber("Limelight Yaw (deg)", botpose[5]);

        SmartDashboard.putNumber("Golden Angle", calculateAngleToHub());

    }

    public double SetTheta() {
        // remember to come back to this
        return 0;
    }

    public Pose2d robotPose2d() {
        return new Pose2d();
    }

    public double distanceToHub() {

        if (isBlue) {
            double diffX = (Hx - Rx);
            double diffY = (blueHy - Ry);
            return Math.hypot(diffX, diffY);
        } else {
            double diffX = (Hx - Rx);
            double diffY = (redHy - Ry);
            return Math.hypot(diffX, diffY);
        }

    }

    public double DistancetoRpms(double distanceInMeters) {
        return 0;
    }

    // red hub is 182.1" from the driver stations and 158.9 from side wall.
    // full field is 651.22"

    // run to golden angle
    // basic stuff, probably could probably be probably made better probably so
    // probably yeah probably uhh probably

    public void zeroPosition() {
        m_turret.setPosition(0);
    }

    public void setPosition(double angle) {
        // private final double position = angle*(ticksPerAngle);
        m_turret.setControl(m_request.withPosition(calculateAngleToHub() * (ticksPerAngle)));
    }

    public double calculateAngleToHub() {
        double theta = m_gyro.getYaw().getValue().in(Degrees);

        double[] defaultPose = new double[6];

        double[] botpose = NetworkTableInstance.getDefault()
                .getTable("limelight-turret")
                .getEntry("botpose")
                .getDoubleArray(defaultPose);

        double Rx = botpose[0];
        double Ry = botpose[1];

        if (isBlue) {
            double diffX = (Hx - Rx);
            double diffY = (blueHy - Ry);

            double turretHubAngle = (Math.toDegrees(Math.atan2(diffX, diffY)));

            return (turretHubAngle-theta);
        } else {
            double diffX = (Hx - Rx);
            double diffY = (redHy - Ry);

            double turretHubAngle = (Math.toDegrees(Math.atan2(diffX, diffY)));

            return (turretHubAngle-theta);
        }
    }

}
//
//
//