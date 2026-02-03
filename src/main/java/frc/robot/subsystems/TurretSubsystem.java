package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.lang.annotation.Target;
import java.util.function.Supplier;

import org.opencv.core.Mat;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurretSubsystem extends SubsystemBase {
    
    private Supplier<Pose2d> poseSupplier;

    private TalonFX turret = new TalonFX(10);
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

    public TurretSubsystem(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        // pid
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
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 140 * (ticksPerAngle);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -140 * (ticksPerAngle);

        // Voltage
        motorConfig.Voltage.PeakForwardVoltage = 16;
        motorConfig.Voltage.PeakReverseVoltage = -16;
        // Differential Constants
        motorConfig.DifferentialConstants.PeakDifferentialDutyCycle = 1;
        motorConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = 800;
        motorConfig.DifferentialConstants.PeakDifferentialVoltage = 16;
        // Motion Magic
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        motorConfig.MotionMagic.MotionMagicAcceleration = 150;
        motorConfig.MotionMagic.MotionMagicExpo_kA = 0.10000000149011612;
        motorConfig.MotionMagic.MotionMagicExpo_kV = 0.11999999731779099;
        // Torque Current
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;

        turret.getConfigurator().apply(motorConfig);

    }

    public void MoveMotor(double targetSpeed) {
        if (turret.getPosition().getValueAsDouble() > -maxAngle
                || turret.getPosition().getValueAsDouble() < maxAngle) {
            turret.set(targetSpeed);
        } else {
            turret.set(0);
        }

    }

    @Override
    public void periodic() {

        robotPose = UpdateRobotPose2d();
        Rx = robotPose.getX();
        Ry = robotPose.getY();
        theta = robotPose.getRotation().getDegrees();

        SmartDashboard.putNumber("tx", LimelightHelpers.getTX("limelight-turret"));
        SmartDashboard.putNumber("ty", LimelightHelpers.getTY("limelight-turret"));

        Pose3d x = LimelightHelpers.getBotPose3d("limelight-turret");

        double[] defaultPose = new double[6];
        double[] botpose = NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("botpose")
                .getDoubleArray(defaultPose);


        SmartDashboard.putNumber("Gyro Angle", theta);
        SmartDashboard.putNumber("Turret Angle", turret.getPosition().getValueAsDouble() / (ticksPerAngle));

        // Push values to SmartDashboard
        SmartDashboard.putNumber("Limelight X (m)", botpose[0]);
        SmartDashboard.putNumber("Limelight Y (m)", botpose[1]);
        SmartDashboard.putNumber("Limelight Z (m)", botpose[2]);
        SmartDashboard.putNumber("Limelight Roll (deg)", botpose[3]);
        SmartDashboard.putNumber("Limelight Pitch (deg)", botpose[4]);
        SmartDashboard.putNumber("Limelight Yaw (deg)", botpose[5]);
        SmartDashboard.putNumber("Golden Angle", calculateAngleToHub());

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

    public void zeroPosition() {
        turret.setPosition(0);
    }

    public void setPosition() {
        
        turret.setControl(m_request.withPosition((calculateAngleToHub()) * (ticksPerAngle)));
    }

    public void setToZero() {

        turret.setControl(m_request.withPosition(-(0) * (ticksPerAngle)));
    }

    public double calculateAngleToHub() {

        double tX = isBlue ? blueHx : redHx;
        double diffY = (Hy - Ry);
        double diffX = (tX - Rx);
        turretHubAngle = Math.toDegrees(Math.atan2(diffY, diffX));
        double goldenAngle = MathUtil.clamp(MathUtil.inputModulus((turretHubAngle - theta), -180, 180), -145, 145); //(turretHubAngle-theta);

        SmartDashboard.putNumber("diffX", diffX);
        SmartDashboard.putNumber("diffY", diffY);
    
        SmartDashboard.putNumber("turretHubAngle", turretHubAngle);
        SmartDashboard.putNumber("Golden Angle", goldenAngle);
        return goldenAngle;
        
    }
}