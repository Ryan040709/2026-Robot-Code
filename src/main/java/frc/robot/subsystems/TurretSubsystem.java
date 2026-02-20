package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//our constants
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    DigitalInput zeroingSensor = new DigitalInput(2); // the magnetic sensor

    private Supplier<Pose2d> poseSupplier;

    private TalonFXS turret = new TalonFXS(10);
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
        TalonFXSConfiguration motorConfig = new TalonFXSConfiguration();
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
        // motorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        // Constants.TurretSubsystem.Turret_PeakForwardTorqueCurrent;
        // motorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        // Constants.TurretSubsystem.Turret_PeakReverseTorqueCurrent;

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

    public void turretSensor() {
        if (zeroingSensor.get()) {
            zeroPosition();
        }
    }

    public void MoveMotor(double targetSpeed) {
        if (!turretLocking) {
            if (turret.getPosition().getValueAsDouble() > -maxAngle
                    || turret.getPosition().getValueAsDouble() < maxAngle && Math.abs(targetSpeed) > 0.1) {
                turret.set(-targetSpeed);
            } else {
                turret.set(0);
            }
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

        turretSensor();

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

        // determine3dOffset(0, 0);

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

    public void setPosition(double velocityX, double velocityY, double targetSpeed) {
        MoveMotor(targetSpeed);
        determine3dOffset(velocityX, velocityY);
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
                lockingTarget = new Translation2d(Hx + (velocityX*ShooterSubsystem.tof), redHubPos.getY() + (velocityY*ShooterSubsystem.tof)); // TODO test this before getting the real robot!
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
                lockingTarget = new Translation2d(Hx + (velocityX*ShooterSubsystem.tof), redHubPos.getY() + (velocityY*ShooterSubsystem.tof)); // so does this system actually work?
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

    Optional<Pose3d> tagPose;

    double fX = 0;
    double fY = 0;
    // not like f(x), "f" just stands for furthest
    double sX = 0;
    double sY = 0;

    double offsetX = 0;
    double offsetY = 0;

    double rotatedX = 0;
    double rotatedY = 0;

    public void determineSwimOffset() {

    }

    public void determine3dOffset(double velocityX, double velocityY) {

        if (tagID >= 1) {

            tagPose = (Constants.AprilTagPositions.aprilTags.getTagPose((int) tagID));

            if (Hy > tagPose.get().getY()) {
                fY = Hy;
                sY = tagPose.get().getY();
            } else if (tagPose.get().getY() > Hy) {
                fY = tagPose.get().getY();
                sY = Hy;
            }

            if (blueHubPos.getX() > tagPose.get().getX()) {
                fX = blueHubPos.getX();
                sX = tagPose.get().getX();
            } else if (tagX > blueHubPos.getX()) {
                fX = tagPose.get().getX();
                sX = blueHubPos.getX();
            }

            offsetX = (fX) - (sX);

            offsetY = (fY) - (sY);

            // offsets are tag-relative, not field relative and should change depending on
            // tag rotation
            if (Math.toDegrees(tagPose.get().getRotation().getZ()) == 180) {
                rotatedX = -offsetX;
                rotatedY = offsetY;
            } else if (Math.toDegrees(tagPose.get().getRotation().getZ()) == 90) {
                rotatedY = offsetX;
                rotatedX = -offsetY;
            } else if (Math.toDegrees(tagPose.get().getRotation().getZ()) == 270) {
                rotatedY = offsetX;
                rotatedX = -offsetY;
            } else if (Math.toDegrees(tagPose.get().getRotation().getZ()) == 0) {
                rotatedX = -offsetX;
                rotatedY = offsetY;
            }

            SmartDashboard.putNumber("tag rotation", Math.toDegrees(tagPose.get().getRotation().getZ()));

            SmartDashboard.putNumber("rotatedX", rotatedX);
            SmartDashboard.putNumber("rotatedY", rotatedY);

            LimelightHelpers.SetFidcuial3DOffset("limelight-turret", rotatedX, rotatedY, 0);
        }
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