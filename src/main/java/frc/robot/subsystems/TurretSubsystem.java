package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.GameManager;

import frc.robot.subsystems.CommandSwerveDrivetrain;

//our constants
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private Supplier<Pose2d> poseSupplier;

    private TalonFX turret = new TalonFX(10);
    private PositionVoltage m_request = new PositionVoltage(0);

    private final double maxAngle = 33.73877 / 90;

    private final double NinetyDegreeRotation = 33.73877;

    Translation2d redHubPos = new Translation2d(11.98482, 4.03606); // red hub

    Translation2d blueHubPos = new Translation2d(4.62554, 4.03606); // blue hub

    Translation2d robotPos = new Translation2d(0, 0); // robot position

    public double redFx = 11.98482 + (4.62554 / 2); // blueFeedingX
    public double blueFx = 4.62554 / 2; // blueFeedingX
    public double rightFy = 2; // right feedingY
    public double leftFy = 6; // left feedingY

    Translation2d lockingTarget = new Translation2d(0, 0); // robot position

    public boolean isFeeding = false;

    public double tagID;
    public Pose2d botPose = new Pose2d();

    public double elapsedTime;

    public PIDController pidRotation = new PIDController(0.0125, 0, 0);
    public double turretRotation;

    public double turretHubAngle = 0;

    public double turretTARGET = 0;

    public double waitTime = 0;

    public List<Integer> blueTagFilter = new ArrayList<>();
    public List<Integer> redTagFilter = new ArrayList<>();

    public double theta = 0;
    public boolean isBlue = true;

    public boolean turretLocking = true;

    private final double rotationsPerDeg = NinetyDegreeRotation / 90;

    public static int kPigeonId = 14;

    private final Pigeon2 m_gyro = new Pigeon2(6, "rio");

    public double robotVelocityX;
    public double robotVelocityY;

    private SwerveDriveOdometry m_odometry;

    public double turretError = turretTARGET - turret.getPosition().getValueAsDouble();

    double txTurret = LimelightHelpers.getTX("limelight-turret");
    double ty = LimelightHelpers.getTY("limelight-tags");
    double ta = LimelightHelpers.getTA("limelight-tags");
    boolean hasTagTargets = LimelightHelpers.getTV("limelight-tags");
    boolean hasTurretTargets = LimelightHelpers.getTV("limelight-turret");

    boolean limelightTurret = false;

    double txnc = LimelightHelpers.getTXNC("limelight-tags"); // Horizontal offset from principal pixel/point to
                                                              // target in degrees
    double tync = LimelightHelpers.getTYNC("limelight-tags"); // Vertical offset from principal pixel/point to target
                                                              // in degrees

    double Hy = isBlue ? blueHubPos.getY() : redHubPos.getY();

    // public double lastTagID = 0;

    public TurretSubsystem(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        // pid
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = Constants.TurretSubsystem.Turret_PeakForwardDutyCycle;
        motorConfig.MotorOutput.PeakReverseDutyCycle = Constants.TurretSubsystem.Turret_PeakReverseDutyCycle;
        // motor "friction" type?
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // regulars
        motorConfig.Slot0.kP = Constants.TurretSubsystem.Turret_Slot0_kP;
        motorConfig.Slot0.kI = Constants.TurretSubsystem.Turret_Slot0_kI;
        motorConfig.Slot0.kD = Constants.TurretSubsystem.Turret_Slot0_kD;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.TurretSubsystem.Turret_StatorCurrentLimitEnable;
        motorConfig.CurrentLimits.StatorCurrentLimit = Constants.TurretSubsystem.Turret_StatorCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.TurretSubsystem.Turret_SupplyCurrentLimitEnable;
        motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretSubsystem.Turret_SupplyCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.TurretSubsystem.Turret_SupplyCurrentLowerLimit;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.TurretSubsystem.Turret_SupplyCurrentLowerTime;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = Constants.TurretSubsystem.Turret_FowardSoftLimitEnable;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 140 * (rotationsPerDeg);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.TurretSubsystem.Turret_ReverseSoftLimitEnable;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -140 * (rotationsPerDeg);
        // Voltage
        motorConfig.Voltage.PeakForwardVoltage = Constants.TurretSubsystem.Turret_PeakForwardVoltage;
        motorConfig.Voltage.PeakReverseVoltage = Constants.TurretSubsystem.Turret_PeakReverseVoltage;
        // Differential Constants
        motorConfig.DifferentialConstants.PeakDifferentialDutyCycle = Constants.TurretSubsystem.Turret_PeakDifferentialDutyCycle;
        motorConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = Constants.TurretSubsystem.Turret_PeakDifferentialDutyCycle;
        motorConfig.DifferentialConstants.PeakDifferentialVoltage = Constants.TurretSubsystem.Turret_PeakDifferentialVoltage;
        // Motion Magic
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretSubsystem.Turret_MotionMagicCruiseVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = Constants.TurretSubsystem.Turret_MotionMagicAcceleration;
        motorConfig.MotionMagic.MotionMagicExpo_kA = Constants.TurretSubsystem.Turret_MotionMagicExpo_kA;
        motorConfig.MotionMagic.MotionMagicExpo_kV = Constants.TurretSubsystem.Turret_MotionMagicExpo_kV;
        // Torque Current
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.TurretSubsystem.Turret_PeakForwardTorqueCurrent;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = Constants.TurretSubsystem.Turret_PeakReverseTorqueCurrent;

        turret.getConfigurator().apply(motorConfig);

        // apriltag filter list
        blueTagFilter.add(18);
        blueTagFilter.add(19);
        blueTagFilter.add(20);
        blueTagFilter.add(21);
        blueTagFilter.add(24);
        blueTagFilter.add(25);
        blueTagFilter.add(26);
        blueTagFilter.add(27);

        redTagFilter.add(2);
        redTagFilter.add(3);
        redTagFilter.add(4);
        redTagFilter.add(5);
        redTagFilter.add(8);
        redTagFilter.add(9);
        redTagFilter.add(10);
        redTagFilter.add(11);

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
        Hy = isBlue ? blueHubPos.getY() : redHubPos.getY();

        robotVelocityX = 0; // CommandSwerveDrivetrain.robotVelocityX;
        robotVelocityY = 0; // CommandSwerveDrivetrain.robotVelocityX;

        isBlue = GameManager.isBlueAlliance;

        hasTurretTargets = LimelightHelpers.getTV("limelight-turret");

        elapsedTime = Timer.getTimestamp();

        botPose = UpdateRobotPose2d();
        robotPos = new Translation2d(botPose.getX(), botPose.getY());
        theta = botPose.getRotation().getDegrees();
        tagID = LimelightHelpers.getFiducialID("limelight-turret");

        SmartDashboard.putBoolean("turret tracking", turretLocking);
        SmartDashboard.putBoolean("is feeding", isFeeding);

        SmartDashboard.putNumber("turret tx", txTurret);

        SmartDashboard.putNumber("tx", LimelightHelpers.getTX("limelight-turret"));

        SmartDashboard.putBoolean("is blue?", isBlue);

        SmartDashboard.putBoolean("limelightTurret", limelightTurret);
        SmartDashboard.putBoolean("turretResults?", hasTurretTargets);

        SmartDashboard.putNumber("Turret Angle", turret.getPosition().getValueAsDouble() / (rotationsPerDeg));

        SmartDashboard.putNumber("Turret Target", turretTARGET + turretError);

        SmartDashboard.putNumber("Angular velocity", m_gyro.getAngularVelocityZWorld().getValueAsDouble());

        //determine3dOffset(velocityX, velocityY);

        // setPosition(robotVelocityX, robotVelocityY);

        SmartDashboard.putNumber("offsetX", offsetX);
        SmartDashboard.putNumber("offsetY", offsetY);

    }

    public Pose2d UpdateRobotPose2d() {
        return poseSupplier.get();
    }

    public void zeroPosition() {
        turret.setPosition(0);
    }

    public boolean turretToggle() {
        if (turretLocking == true) {
            turretLocking = false;
        } else if (!turretLocking) {
            turretLocking = true;
        }
        return turretLocking;
    }

    public boolean FilterApriltags() {

        if (isBlue) {
            if (blueTagFilter.contains((int) tagID)) {
                return true;
            } else {
                return false;
            }
        } else {
            if (redTagFilter.contains((int) tagID)) {
                return true;
            } else {
                return false;
            }
        }
    }

    public void setPosition(double velocityX, double velocityY) {
        FilterApriltags();

        double feedforward = -(m_gyro.getAngularVelocityZWorld().getValueAsDouble());

        double lastTagID = 0;

        if (turretLocking) {

            if (hasTurretTargets == true && FilterApriltags()) {
                limelightTurret = true;

                if (elapsedTime > waitTime + 1 && !isFeeding) {
                    if (hasTurretTargets == true) {

                        turret.setControl(m_request
                                .withPosition(
                                        (turret.getPosition().getValueAsDouble() + -txTurret * (rotationsPerDeg))));

                        turretTARGET = turret.getPosition().getValueAsDouble() + -txTurret * (rotationsPerDeg);
                    }
                } else {
                    lastTagID = tagID;
                    turret.setControl(
                            m_request.withPosition((calculateAngleToHub(velocityX, velocityY) * (rotationsPerDeg))));
                }

            } else {

                waitTime = elapsedTime;

                turret.setControl(
                        m_request.withPosition((calculateAngleToHub(velocityX, velocityY) * (rotationsPerDeg))));
                limelightTurret = false;

                turretTARGET = (calculateAngleToHub(velocityX, velocityY) * (rotationsPerDeg));

            }

            SmartDashboard.putBoolean("filtered tag?", FilterApriltags());

            SmartDashboard.putNumber("feed forward", feedforward);

            SmartDashboard.putNumber("lastTagID", lastTagID);
            txTurret = LimelightHelpers.getTX("limelight-turret");

        } else {
            // nothing???????????????????????????????????????????????????????? but
            // whyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
        }

    }

    public void setToZero() {

        turret.setControl(m_request.withPosition(-(0) * (rotationsPerDeg)));

    }

    public void determineLockingTarget(double velocityX, double velocityY) {

        double tX = isBlue ? blueFx : redFx;
        double Hx = isBlue ? blueHubPos.getX() : redHubPos.getX();

        if (isBlue) {
            if (robotPos.getX() > blueHubPos.getX()) {
                if (robotPos.getY() > Hy) {
                    lockingTarget = new Translation2d(tX, leftFy);
                } else if (robotPos.getY() < Hy) {
                    lockingTarget = new Translation2d(tX, rightFy);
                }
                isFeeding = true;
            } else {
                lockingTarget = new Translation2d(Hx, redHubPos.getY());
                isFeeding = false;
            }
        } else {
            if (robotPos.getX() < redHubPos.getX()) {
                if (robotPos.getY() > Hy) {
                    lockingTarget = new Translation2d(tX, leftFy);
                } else if (robotPos.getY() < Hy) {
                    lockingTarget = new Translation2d(tX, rightFy);
                }
                isFeeding = true;
            } else {
                lockingTarget = new Translation2d(Hx - velocityX, Hy - velocityY);
                isFeeding = false;
            }
        }

        SmartDashboard.putNumber("offset X", velocityX);
        SmartDashboard.putNumber("offset Y", velocityY);

        SmartDashboard.putNumber("targetX", lockingTarget.getX());
        SmartDashboard.putNumber("targetY", lockingTarget.getY());
    }

    double tagX = 0;

    double tagY = 0;

    double tagRotation; // certain values must change to accomadate apriltag orientation

    double fX = 0;
    double fY = 0;
    // not like f(x), "f" just stands for furthest
    double sX = 0;
    double sY = 0;

    double offsetX = 0;
    double offsetY = 0;

    double rotatedX = 0;
    double rotatedY = 0;

    public void determine3dOffset(double velocityX, double velocityY) {

            //tagRotation = (Constants.AprilTagPositions.aprilTags.getTagRotation((int) tagID));

        if (tagID == 18) {
            tagX = (Constants.AprilTagPositions.Tag18X / 39.37);
            tagY = (Constants.AprilTagPositions.Tag18Y / 39.37);
            tagRotation = Constants.AprilTagPositions.Tag18Rotation;
        } else if (tagID == 19) {
            tagX = (Constants.AprilTagPositions.Tag19X / 39.37);
            tagY = (Constants.AprilTagPositions.Tag19Y / 39.37);
        } else if (tagID == 20) {
            tagX = (Constants.AprilTagPositions.Tag20X / 39.37);
            tagY = (Constants.AprilTagPositions.Tag20Y / 39.37);
            tagRotation = Constants.AprilTagPositions.Tag20Rotation;
        } else if (tagID == 21) {
            tagX = (Constants.AprilTagPositions.Tag21X / 39.37);
            tagY = (Constants.AprilTagPositions.Tag21Y / 39.37);
            tagRotation = Constants.AprilTagPositions.Tag21Rotation;
        } else if (tagID == 22) {
            tagX = (Constants.AprilTagPositions.Tag22X / 39.37);
            tagY = (Constants.AprilTagPositions.Tag22Y / 39.37);
        } else if (tagID == 24) {
            tagX = (Constants.AprilTagPositions.Tag24X / 39.37);
            tagY = (Constants.AprilTagPositions.Tag24Y / 39.37);
            tagRotation = Constants.AprilTagPositions.Tag24Rotation;
        } else if (tagID == 25) {
            tagX = (Constants.AprilTagPositions.Tag25X / 39.37);
            tagY = (Constants.AprilTagPositions.Tag25Y / 39.37);
            tagRotation = Constants.AprilTagPositions.Tag25Rotation;
        } else if (tagID == 26) {
            tagX = (Constants.AprilTagPositions.Tag26X / 39.37);
            tagY = (Constants.AprilTagPositions.Tag26Y / 39.37);
            tagRotation = Constants.AprilTagPositions.Tag26Rotation;
        } else if (tagID == 27) {
            tagX = (Constants.AprilTagPositions.Tag27X / 39.37);
            tagY = (Constants.AprilTagPositions.Tag27Y / 39.37);
            tagRotation = Constants.AprilTagPositions.Tag27Rotation;
        }

        if (Hy > tagY) {
            fY = Hy;
            sY = tagY;
        } else if (tagY > Hy) {
            fY = tagY;
            sY = Hy;
        }

        if (blueHubPos.getX() > tagX) {
            fX = blueHubPos.getX();
            sX = tagX;
        } else if (tagX > blueHubPos.getX()) {
            fX = tagX;
            sX = blueHubPos.getX();
        }

        offsetX = (fX) - (sX);

        offsetY = (fY) - (sY);

        // offsets are tag-relative, not field relative and should change depending on
        // tag rotation
        if (tagRotation == 180) {
            rotatedX = -offsetX;
            rotatedY = offsetY;
        } else if (tagRotation == 90) {
            rotatedY = -offsetX;
            rotatedX = -offsetY;
        } else if (tagRotation == 270) {
            rotatedY = offsetX;
            rotatedX = -offsetY;
        } else if (tagRotation == 0) {
            rotatedX = -offsetX;
            rotatedY = offsetY;
        }



        SmartDashboard.putNumber("rotatedX", rotatedX);
        SmartDashboard.putNumber("rotatedY", rotatedY);

        LimelightHelpers.SetFidcuial3DOffset("limelight-turret", rotatedX, rotatedY, 0);
    }

    public double calculateAngleToHub(double velocityX, double velocityY) {

        determineLockingTarget(velocityX, velocityY);

        double diffY = (lockingTarget.getY() - robotPos.getY());
        double diffX = (lockingTarget.getX() - robotPos.getX());
        turretHubAngle = Math.toDegrees(Math.atan2(diffY, diffX));
        double goldenAngle = MathUtil.clamp(MathUtil.inputModulus((turretHubAngle - theta), -180, 180), -145, 145); // (turretHubAngle-theta);

        SmartDashboard.putNumber("diffX", diffX);
        SmartDashboard.putNumber("diffY", diffY);

        SmartDashboard.putNumber("turretHubAngle", turretHubAngle);
        SmartDashboard.putNumber("Golden Angle", goldenAngle);

        turretError = turretTARGET - turret.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Turret Error", turretError);
        SmartDashboard.putNumber("Turret Position", turret.getPosition().getValueAsDouble());

        return goldenAngle;

    }
}