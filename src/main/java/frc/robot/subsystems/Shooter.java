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

public class Shooter extends SubsystemBase {

    private Supplier<Pose2d> poseSupplier;

    private TalonFX m_ShooterL = new TalonFX(15);
    private TalonFX m_ShooterR = new TalonFX(16);

    private TalonFX m_Hood = new TalonFX(17);
    private PositionVoltage m_request = new PositionVoltage(0);

    private final double maxAngle = 33.73877 / 90;

    private final double NinetyDegreeRotation = 33.73877;

    public final double Hy = 4.03606;
    public final double redHx = 11.98482;
    public final double blueHx = 4.62554;
    public double Rx;
    public double Ry;
    public Pose2d robotPose = new Pose2d();

    public double turretHubAngle = 0;

    public double theta = 0;
    public boolean isBlue = true;

    public boolean isToggle = true;

    private final double ticksPerAngle = NinetyDegreeRotation / 90;

    public static int kPigeonId = 14;

    private final Pigeon2 m_gyro = new Pigeon2(6, "rio");

    // setting the postions of our swerve modules for kinematics
    private Translation2d m_frontLeftLocation = new Translation2d(.3429, .3429);
    private Translation2d m_frontRightLocation = new Translation2d(.3429, -.3429);
    private Translation2d m_backLeftLocation = new Translation2d(-.3429, .3429);
    private Translation2d m_backRightLocation = new Translation2d(-.3429, -.3429);

    private SwerveDriveOdometry m_odometry;

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
            m_backRightLocation);

    // Basic targeting data
    double tx = LimelightHelpers.getTX("limelight-turret"); // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("limelight-turret"); // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("limelight-turret"); // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV("limelight-turret"); // Do you have a valid target?

    double txnc = LimelightHelpers.getTXNC("limelight-turret"); // Horizontal offset from principal pixel/point to
                                                                // target in degrees
    double tync = LimelightHelpers.getTYNC("limelight-turret"); // Vertical offset from principal pixel/point to target
                                                                // in degrees
    VelocityVoltage velocity = new VelocityVoltage(0);

    InvertedValue Invert = InvertedValue.Clockwise_Positive;
    NeutralModeValue Coast = NeutralModeValue.Coast;
    NeutralModeValue Break = NeutralModeValue.Brake;

    public Shooter(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        //pid
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.PeakForwardDutyCycle = 1;
        shooterConfig.MotorOutput.PeakReverseDutyCycle = -1;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterConfig.Slot0.kP = 1;
        shooterConfig.Slot0.kI = 0.15;
        shooterConfig.Slot0.kD = 0;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.StatorCurrentLimit = 100;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 100;
        shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        shooterConfig.CurrentLimits.SupplyCurrentLowerTime = -40;
        shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        shooterConfig.Voltage.PeakForwardVoltage = 16;
        shooterConfig.Voltage.PeakReverseVoltage = -16;
        // Differential Constants
        shooterConfig.DifferentialConstants.PeakDifferentialDutyCycle = 1;
        shooterConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = 800;
        shooterConfig.DifferentialConstants.PeakDifferentialVoltage = 16;
        // Motion Magic
        shooterConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        shooterConfig.MotionMagic.MotionMagicAcceleration = 150;
        shooterConfig.MotionMagic.MotionMagicExpo_kA = 0.10000000149011612;
        shooterConfig.MotionMagic.MotionMagicExpo_kV = 0.11999999731779099;
        // Torque Current
        shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
        shooterConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
        // Set shooter motor settings
        shooterConfig.MotorOutput.Inverted = Invert;
        shooterConfig.MotorOutput.PeakForwardDutyCycle = .5;
        shooterConfig.MotorOutput.PeakReverseDutyCycle = -.5;
        shooterConfig.MotorOutput.NeutralMode = Coast;

        m_ShooterL.getConfigurator().apply(shooterConfig);
        m_ShooterR.getConfigurator().apply(shooterConfig);

        // hood motor PID
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.PeakForwardDutyCycle = 1;
        hoodConfig.MotorOutput.PeakReverseDutyCycle = -1;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.Slot0.kP = 1;
        hoodConfig.Slot0.kI = 0.15;
        hoodConfig.Slot0.kD = 0;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 100;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 100;
        hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        hoodConfig.CurrentLimits.SupplyCurrentLowerTime = -40;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 140 * (ticksPerAngle);
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -140 * (ticksPerAngle);

        // Voltage
        hoodConfig.Voltage.PeakForwardVoltage = 16;
        hoodConfig.Voltage.PeakReverseVoltage = -16;
        // Differential Constants
        hoodConfig.DifferentialConstants.PeakDifferentialDutyCycle = 1;
        hoodConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = 800;
        hoodConfig.DifferentialConstants.PeakDifferentialVoltage = 16;
        // Motion Magic
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        hoodConfig.MotionMagic.MotionMagicAcceleration = 150;
        hoodConfig.MotionMagic.MotionMagicExpo_kA = 0.10000000149011612;
        hoodConfig.MotionMagic.MotionMagicExpo_kV = 0.11999999731779099;
        // Torque Current
        hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
        hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;

        m_Hood.getConfigurator().apply(hoodConfig);

    }

    public void zeroHood() {
        m_Hood.setPosition(0);
    }

    public void setHoodPosition() {
        m_Hood.setPosition(calculateHoodPosition());
    }

    public void RuntoRPMs() {
        m_ShooterL.setControl(velocity.withVelocity(DistancetoRpms(calculateDistanceToHub())));
        m_ShooterR.setControl(velocity.withVelocity(-DistancetoRpms(calculateDistanceToHub())));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance To Target", calculateDistanceToHub());
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

        double targetRPM = (1 / 2) * (calculateDistanceToHub()) + 10; // the slope is a placeholder
        // y=mx+b where "y" is the target RPM and "x" is the distance between the robot
        // and target
        // to find the slope, determine positions and rpms that we know work on certain
        // spots on the field, and create a line of best fit.

        return targetRPM;
    }

    public double calculateDistanceToHub() {

        double tX = isBlue ? blueHx : redHx;
        double DistanceToTarget = Math.sqrt((Math.pow((Rx - tX), 2) + Math.pow((Ry - Hy), 2)));
        // uses point distance formula to determine the distance between the target and
        // robot

        SmartDashboard.putNumber("Distance To Target", DistanceToTarget);
        return DistanceToTarget;

    }

    public double calculateHoodPosition() {

        double zoneOne = 10; // TODO: change to actual zone number

        double zoneTwo = 20; // TODO: change to actual zone number

        double zoneThree = 30; // TODO: change to actual zone number

        double zoneFour = 40; // TODO: change to actual zone number

        double zoneFive = 50; // TODO: change to actual zone number

        // hood positions
        double zoneOnePosition = 10; // TODO: change to actual zone number

        double zoneTwoPosition = 20; // TODO: change to actual zone number

        double zoneThreePosition = 30; // TODO: change to actual zone number

        double zoneFourPosition = 40; // TODO: change to actual zone number

        double zoneFivePosition = 50; // TODO: change to actual zone number

        // get the target distance
        double distanceToTarget = calculateDistanceToHub();

        double targetHoodPosition = 0;

        if (zoneOne <= distanceToTarget) {
            targetHoodPosition = zoneOnePosition;
        } else if (zoneTwo <= distanceToTarget) {
            targetHoodPosition = zoneTwoPosition;
        } else if (zoneThree <= distanceToTarget) {
            targetHoodPosition = zoneThreePosition;
        } else if (zoneFour <= distanceToTarget) {
            targetHoodPosition = zoneFourPosition;
        } else if (zoneFive <= distanceToTarget) {
            targetHoodPosition = zoneFivePosition;
        }

        return targetHoodPosition;

    }
}