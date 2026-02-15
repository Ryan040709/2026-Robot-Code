package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.lang.annotation.Target;
import java.util.function.Supplier;

import org.opencv.core.Mat;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ShooterSubsystem extends SubsystemBase {

    private Supplier<Pose2d> poseSupplier;

    private TalonFX shooterMotorL = new TalonFX(15);
    private TalonFX shooterMotorR = new TalonFX(16);

    private TalonFX hood = new TalonFX(17);
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

    private final Pigeon2 gyro = new Pigeon2(6, "rio");

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

    public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        // pid
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.PeakForwardDutyCycle = Constants.ShooterSubsystem.Shooter_PeakForwardDutyCycle;
        shooterConfig.MotorOutput.PeakReverseDutyCycle = Constants.ShooterSubsystem.Shooter_PeakReverseDutyCycle;
        // motor "friction" type?
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // regulars
        shooterConfig.Slot0.kP = Constants.ShooterSubsystem.Shooter_Slot0_kP;
        shooterConfig.Slot0.kI = Constants.ShooterSubsystem.Shooter_Slot0_kI;
        shooterConfig.Slot0.kD = Constants.ShooterSubsystem.Shooter_Slot0_kD;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.ShooterSubsystem.Shooter_StatorCurrentLimitEnable;
        shooterConfig.CurrentLimits.StatorCurrentLimit = Constants.ShooterSubsystem.Shooter_CurrentLimit;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ShooterSubsystem.Shooter_SupplyCurrentLimitEnable;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = Constants.ShooterSubsystem.Shooter_SupplyCurrentLimit;
        shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.ShooterSubsystem.Shooter_SupplyCurrentLowerLimit;
        shooterConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.ShooterSubsystem.Shooter_SupplyCurrentLowerTime;
        shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = Constants.ShooterSubsystem.Shooter_FowardSoftLimitEnable;
        shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 140;
        shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.ShooterSubsystem.Shooter_ReverseSoftLimitEnable;
        shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -140;
        // Voltage
        shooterConfig.Voltage.PeakForwardVoltage = Constants.ShooterSubsystem.Shooter_PeakForwardVoltage;
        shooterConfig.Voltage.PeakReverseVoltage = Constants.ShooterSubsystem.Shooter_PeakReverseVoltage;
        // Differential Constants
        shooterConfig.DifferentialConstants.PeakDifferentialDutyCycle = Constants.ShooterSubsystem.Shooter_PeakDifferentialDutyCycle;
        shooterConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = Constants.ShooterSubsystem.Shooter_PeakDifferentialDutyCycle;
        shooterConfig.DifferentialConstants.PeakDifferentialVoltage = Constants.ShooterSubsystem.Shooter_PeakDifferentialVoltage;
        // Motion Magic
        shooterConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ShooterSubsystem.Shooter_MotionMagicCruiseVelocity;
        shooterConfig.MotionMagic.MotionMagicAcceleration = Constants.ShooterSubsystem.Shooter_MotionMagicAcceleration;
        shooterConfig.MotionMagic.MotionMagicExpo_kA = Constants.ShooterSubsystem.Shooter_MotionMagicExpo_kA;
        shooterConfig.MotionMagic.MotionMagicExpo_kV = Constants.ShooterSubsystem.Shooter_MotionMagicExpo_kV;
        // Torque Current
        shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.ShooterSubsystem.Shooter_PeakForwardTorqueCurrent;
        shooterConfig.TorqueCurrent.PeakReverseTorqueCurrent = Constants.ShooterSubsystem.Shooter_PeakReverseTorqueCurrent;

        shooterMotorL.getConfigurator().apply(shooterConfig);
        shooterMotorR.getConfigurator().apply(shooterConfig);

        // hood motor PID
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
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
        hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.ShooterSubsystem.Hood_SupplyCurrentLowerLimit;
        hoodConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.ShooterSubsystem.Hood_SupplyCurrentLowerTime;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = Constants.ShooterSubsystem.Hood_FowardSoftLimitEnable;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 140;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.ShooterSubsystem.Hood_ReverseSoftLimitEnable;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -140;
        // Voltage
        hoodConfig.Voltage.PeakForwardVoltage = Constants.ShooterSubsystem.Hood_PeakForwardVoltage;
        hoodConfig.Voltage.PeakReverseVoltage = Constants.ShooterSubsystem.Hood_PeakReverseVoltage;
        // Differential Constants
        hoodConfig.DifferentialConstants.PeakDifferentialDutyCycle = Constants.ShooterSubsystem.Hood_PeakDifferentialDutyCycle;
        hoodConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = Constants.ShooterSubsystem.Hood_PeakDifferentialDutyCycle;
        hoodConfig.DifferentialConstants.PeakDifferentialVoltage = Constants.ShooterSubsystem.Hood_PeakDifferentialVoltage;
        // Motion Magic
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ShooterSubsystem.Hood_MotionMagicCruiseVelocity;
        hoodConfig.MotionMagic.MotionMagicAcceleration = Constants.ShooterSubsystem.Hood_MotionMagicAcceleration;
        hoodConfig.MotionMagic.MotionMagicExpo_kA = Constants.ShooterSubsystem.Hood_MotionMagicExpo_kA;
        hoodConfig.MotionMagic.MotionMagicExpo_kV = Constants.ShooterSubsystem.Hood_MotionMagicExpo_kV;
        // Torque Current
        hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.ShooterSubsystem.Hood_PeakForwardTorqueCurrent;
        hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = Constants.ShooterSubsystem.Hood_PeakReverseTorqueCurrent;

        hood.getConfigurator().apply(hoodConfig);

        shooterMotorR.setControl(new Follower(15, MotorAlignmentValue.Opposed));

    }

    public void zeroHood() {
        hood.setPosition(0);
    }

    public void setHoodPosition() {
        hood.setPosition(calculateHoodPosition());
    }

    public void RuntoRPMs() {
        shooterMotorL.setControl(velocity.withVelocity(DistancetoRpms(calculateDistanceToHub())));
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