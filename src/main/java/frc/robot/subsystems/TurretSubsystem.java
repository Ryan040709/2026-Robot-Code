package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.lang.annotation.Target;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.CommandSwerveDrivetrain;

//our constants
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private Supplier<Pose2d> poseSupplier;

    private TalonFX turret = new TalonFX(10);
    private PositionVoltage m_request = new PositionVoltage(0);

    private final double maxAngle = 33.73877 / 90;

    private final double NinetyDegreeRotation = 33.73877;

    public final double Hy = 4.03606; // hubY
    public final double redHx = 11.98482; // redHubX
    public final double blueHx = 4.62554; // blueHubX
    public double Rx; // robotX
    public double Ry; // robotY

    public double redFx = 11.98482 + (4.62554/2); // blueFeedingX
    public double blueFx = 4.62554/2; // blueFeedingX
    public double rightFy = 2; // right feedingY
    public double leftFy = 6; // left feedingY

    public double Lx; // lockingX
    public double Ly; // lockingY

    public boolean isFeeding = false;

    public double tagID;
    public Pose2d robotPose = new Pose2d();

    public double elapsedTime;

    public PIDController pidRotation = new PIDController(0.0125, 0, 0);
    public double turretRotation;

    public double turretHubAngle = 0;

    public double turretTARGET = 0;

    public double waitTime = 0;

    public double theta = 0;
    public boolean isBlue = true;

    public boolean turretLocking = true;

    private final double ticksPerAngle = NinetyDegreeRotation / 90;

    public static int kPigeonId = 14;

    private final Pigeon2 m_gyro = new Pigeon2(6, "rio");

    private SwerveDriveOdometry m_odometry;

    // double txTURRET = LimelightHelpers.getTX("limelight-tags");

    // double tagID = LimelightHelpers.getFiducialID("limelight-tags");

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

    public TurretSubsystem(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        // SmartDashboard.puttTable("limelight-left").getEntry("tv").getDouble(0) == 1

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
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 140 * (ticksPerAngle);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.TurretSubsystem.Turret_ReverseSoftLimitEnable;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -140 * (ticksPerAngle);
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

        Optional<Alliance> ally = DriverStation.getAlliance();
if (ally.isPresent()) {
    if (ally.get() == Alliance.Red) {
        isBlue = false;
    }
    if (ally.get() == Alliance.Blue) {
        isBlue = true;
    }
}
else {
    isBlue = false;
}

        hasTurretTargets = LimelightHelpers.getTV("limelight-turret");

        elapsedTime = Timer.getTimestamp();

        // i really don't know :/

        // hasTurretTargets =
        // NetworkTableInstance.getDefault().getTable("limelight-tags").getEntry("tv").getDouble(0);

        // LimelightHelpers.SetFidcuial3DOffset("limelight-tags", 1, 1, 1);

        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret",
        // 25,27,24,20);

        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", int[25]);;

        robotPose = UpdateRobotPose2d();
        Rx = robotPose.getX();
        Ry = robotPose.getY();
        theta = robotPose.getRotation().getDegrees();
        tagID = LimelightHelpers.getFiducialID("limelight-turret");

        // double tagID = LimelightHelpers.getFiducialID("limelight-tags");

        SmartDashboard.putNumber("tagID", tagID);

        SmartDashboard.putBoolean("turret tracking", turretLocking);
        SmartDashboard.putBoolean("is feeding", isFeeding);

        SmartDashboard.putNumber("wait time", waitTime);

        // SmartDashboard.putNumber("turretResults", hasTurretTargets);

        SmartDashboard.putNumber("turret tx", txTurret);

        SmartDashboard.putNumber("tx", LimelightHelpers.getTX("limelight-turret"));
        SmartDashboard.putNumber("ty", LimelightHelpers.getTY("limelight-tags"));

        Pose3d x = LimelightHelpers.getBotPose3d("limelight-tags");

        double[] defaultPose = new double[6];
        double[] botpose = NetworkTableInstance.getDefault().getTable("limelight-tags").getEntry("botpose")
                .getDoubleArray(defaultPose);

        SmartDashboard.putNumber("Gyro Angle", theta);

        SmartDashboard.putBoolean("is blue?", isBlue);

        SmartDashboard.putBoolean("limelightTurret", limelightTurret);
        SmartDashboard.putBoolean("turretResults?", hasTurretTargets);

        SmartDashboard.putNumber("Turret Angle", turret.getPosition().getValueAsDouble() / (ticksPerAngle));

        // Push values to SmartDashboard
        SmartDashboard.putNumber("Limelight X (m)", botpose[0]);
        SmartDashboard.putNumber("Limelight Y (m)", botpose[1]);
        SmartDashboard.putNumber("Limelight Z (m)", botpose[2]);
        SmartDashboard.putNumber("Limelight Roll (deg)", botpose[3]);
        SmartDashboard.putNumber("Limelight Pitch (deg)", botpose[4]);
        SmartDashboard.putNumber("Limelight Yaw (deg)", botpose[5]);
        SmartDashboard.putNumber("Golden Angle", calculateAngleToHub());

        SmartDashboard.putNumber("Turret Target", turretTARGET);

        SmartDashboard.putNumber("Time", elapsedTime);

        determine3dOffset();

        setPosition();

        // if (tagID == 19) {
        // LimelightHelpers.SetFidcuial3DOffset("limelight-tags", 0, 0, 0);
        // } else if (tagID == 25) {
        // LimelightHelpers.SetFidcuial3DOffset("limelight-tags", 1, 1, 0);
        // }

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

    public boolean turretToggle() {
        if (turretLocking == true) {
            turretLocking = false;
        } else if (!turretLocking) {
            turretLocking = true;
        }
        return turretLocking;
    }

    public void setPosition() {

        double lastTagID = 0;

        if (turretLocking) {

            if (hasTurretTargets == true) {
                limelightTurret = true;

                if (elapsedTime > waitTime + 0.25 && !isFeeding && lastTagID == tagID) {
                    if (hasTurretTargets == true) {

                        turret.setControl(m_request
                                .withPosition(turret.getPosition().getValueAsDouble() + -txTurret * (ticksPerAngle)));

                        turretTARGET = turret.getPosition().getValueAsDouble() + -txTurret * (ticksPerAngle);
                    }
                } else {
                    lastTagID = tagID;
                    turret.setControl(m_request.withPosition((calculateAngleToHub() * (ticksPerAngle))));
                }

            } else {

                waitTime = elapsedTime;

                turret.setControl(m_request.withPosition((calculateAngleToHub() * (ticksPerAngle))));
                limelightTurret = false;

                turretTARGET = (calculateAngleToHub() * (ticksPerAngle));

            }

            SmartDashboard.putNumber("lastTagID", lastTagID);
            txTurret = LimelightHelpers.getTX("limelight-turret");

        } else {
            // nothing???????????????????????????????????????????????????????? but
            // whyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
        }

    }

    public void setToZero() {

        turret.setControl(m_request.withPosition(-(0) * (ticksPerAngle)));

    }

    public void determineLockingTarget() {

        double tX = isBlue ? blueFx : redFx;
        double Hx = isBlue ? blueHx : redHx;


if (isBlue) {
        if (Rx > blueHx) {
            if (Ry > Hy) {
                Ly = leftFy;
                Lx = tX;
            } else if (Ry < Hy) {
                Ly = rightFy;
                Lx = tX;
            }
            isFeeding = true;
        } else {
            Lx = Hx;
            Ly = Hy;
            isFeeding = false;
        }
    }
else {
        if (Rx < redHx) {
            if (Ry > Hy) {
                Ly = leftFy;
                Lx = tX;
            } else if (Ry < Hy) {
                Ly = rightFy;
                Lx = tX;
            }
            isFeeding = true;
        } else {
            Lx = Hx;
            Ly = Hy;
            isFeeding = false;
        }
}

        SmartDashboard.putNumber("targetX", Lx);
        SmartDashboard.putNumber("targetY", Ly);
    }

    public void determine3dOffset() {

        double tagX = 0;
        double tagY = 0;

        double tagRotation = 0; // certain values must change to accomadate apriltag orientation

        double fX = 0;
        double fY = 0;
        // not like f(x), "f" just stands for furthest
        double sX = 0;
        double sY = 0;

        double offsetX = 0;
        double offsetY = 0;

        double rotatedX = 0;
        double rotatedY = 0;
        // rotated offsets so they can be tag-relative

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

        if (blueHx > tagX) {
            fX = blueHx;
            sX = tagX;
        } else if (tagX > blueHx) {
            fX = tagX;
            sX = blueHx;
        }

        offsetX = fX - sX;

        offsetY = fY - sY;

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

        SmartDashboard.putNumber("TagRotation", tagRotation);
        SmartDashboard.putNumber("offsetX", offsetX);
        SmartDashboard.putNumber("offsetY", offsetY);

        SmartDashboard.putNumber("rotatedX", rotatedX);
        SmartDashboard.putNumber("rotatedY", rotatedY);

        LimelightHelpers.SetFidcuial3DOffset("limelight-turret", rotatedX, rotatedY, 0);
    }

    public void SetPid() {
        pidRotation.setPID(0.02, 0.0, 0);
        turretRotation = pidRotation.calculate(txTurret, 0);
    }

    public double calculateAngleToHub() {

        determineLockingTarget();

        double tX = isBlue ? blueHx : redHx;
        double diffY = (Ly - Ry);
        double diffX = (Lx - Rx);
        turretHubAngle = Math.toDegrees(Math.atan2(diffY, diffX));
        double goldenAngle = MathUtil.clamp(MathUtil.inputModulus((turretHubAngle - theta), -180, 180), -145, 145); // (turretHubAngle-theta);

        SmartDashboard.putNumber("diffX", diffX);
        SmartDashboard.putNumber("diffY", diffY);

        SmartDashboard.putNumber("turretHubAngle", turretHubAngle);
        SmartDashboard.putNumber("Golden Angle", goldenAngle);
        return goldenAngle;

    }
}