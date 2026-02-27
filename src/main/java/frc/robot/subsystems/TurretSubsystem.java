package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
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

    Translation2d redHubPos = new Translation2d(11.98482, 4.03606); // red hub

    Translation2d blueHubPos = new Translation2d(4.62554, 4.03606); // blue hub

    Translation2d robotPos = new Translation2d(0, 0); // robot position

    public double redFx = 11.98482 + (4.62554 / 2); // blueFeedingX
    public double blueFx = 4.62554 / 2; // blueFeedingX
    public double rightFy = 2; // right feedingY
    public double rightFyFar = 0.5; // right feedingY
    public double leftFy = 6; // left feedingY
    public double leftFyFar = 7.5; // left feedingY

    Translation2d lockingTarget = new Translation2d(0, 0); // robot position

    public boolean isFeeding = true;

    public double tagID;
    public Pose2d botPose = new Pose2d();

    public double elapsedTime;

    public double turretRotation;

    public double turretHubAngle = 0;

    public double turretTARGET = 0;

    public double waitTime = 0;

    public List<Integer> blueTagFilter = new ArrayList<>();
    public List<Integer> redTagFilter = new ArrayList<>();

    public double theta = 0;
    public boolean isBlue = true;

    public boolean turretLocking = true;

    public double turretError = turretTARGET - turret.getPosition().getValueAsDouble();

    double txTurret = LimelightHelpers.getTX("limelight-turret");
    boolean hasTurretTargets = LimelightHelpers.getTV("limelight-turret");

    boolean limelightTurret = false;

    double txnc = LimelightHelpers.getTXNC("limelight-tags"); // Horizontal offset from principal pixel/point to
                                                              // target in degrees
    double tync = LimelightHelpers.getTYNC("limelight-tags"); // Vertical offset from principal pixel/point to target
                                                              // in degrees

    double Hy = isBlue ? blueHubPos.getY() : redHubPos.getY();

    PIDController turretPID = new PIDController(1, 1, 1);

    public TurretSubsystem(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        // pid
        TalonFXSConfiguration motorConfig = new TalonFXSConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = Constants.TurretSubsystem.Turret_PeakForwardDutyCycle;
        motorConfig.MotorOutput.PeakReverseDutyCycle = Constants.TurretSubsystem.Turret_PeakReverseDutyCycle;
        // motor "friction" type?
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
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
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.TurretSubsystem.Turret_FowardSoftLimit;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.TurretSubsystem.Turret_ReverseSoftLimitEnable;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.TurretSubsystem.Turret_ReverseSoftLimit;
        // Voltage
        motorConfig.Voltage.PeakForwardVoltage = Constants.TurretSubsystem.Turret_PeakForwardVoltage;
        motorConfig.Voltage.PeakReverseVoltage = Constants.TurretSubsystem.Turret_PeakReverseVoltage;
        // gear ratio
        motorConfig.ExternalFeedback.SensorToMechanismRatio = Constants.TurretSubsystem.Turret_SensorToMechanismRatio;

        motorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        turret.getConfigurator().apply(motorConfig);

        // apriltag filter list
        blueTagFilter.add(18);
        // blueTagFilter.add(19);
        blueTagFilter.add(20);
        blueTagFilter.add(21);
        // blueTagFilter.add(24);
        // blueTagFilter.add(25);
        blueTagFilter.add(26);
        // blueTagFilter.add(27);

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
        if (!zeroingSensor.get()) {
            // zeroPosition();
        }
    }

    public void MoveMotor(double targetSpeed) {
        if (!turretLocking) {
            if (Math.abs(targetSpeed) > 0.1) {
                turret.set(-targetSpeed);
            } else {
                turret.set(0);
            }
        }

    }

    @Override
    public void periodic() {
        Hy = isBlue ? blueHubPos.getY() : redHubPos.getY();
        isBlue = GameManager.isBlueAlliance;
        hasTurretTargets = LimelightHelpers.getTV("limelight-turret");
        elapsedTime = Timer.getTimestamp();
        botPose = UpdateRobotPose2d();
        robotPos = new Translation2d(botPose.getX(), botPose.getY());
        theta = botPose.getRotation().getDegrees();
        tagID = LimelightHelpers.getFiducialID("limelight-turret");

        turretSensor();

        SmartDashboard.putBoolean("is feeding", isFeeding);
        SmartDashboard.putNumber("Turret Angle", turret.getPosition().getValueAsDouble());

        SmartDashboard.putBoolean("has turret targets?", hasTurretTargets);
        SmartDashboard.putBoolean("tag is in filter list?", FilterApriltags());

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

    public void setPosition(double velocityX, double velocityY, double targetSpeed, Translation2d robotpose) {
        MoveMotor(targetSpeed);
        determine3dOffset(velocityX, velocityY);
        FilterApriltags();

        if (turretLocking) {

            determineLockingMethod(velocityX, velocityY, robotpose);

            txTurret = LimelightHelpers.getTX("limelight-turret");

            SmartDashboard.putNumber("Turret Position", turret.getPosition().getValueAsDouble());

        }

    }

    public void determineLockingMethod(double velocityX, double velocityY, Translation2d robotpose) {
        if (hasTurretTargets == true && FilterApriltags()) {
            if (elapsedTime > waitTime + 1 && !isFeeding) {
                turret.set(calculateTurretPID(velocityX, velocityY, robotpose));
            } else {
                turret.setControl(m_request.withPosition((calculateAngleToHub(velocityX, velocityY, robotPos))));
            }
        } else {
            waitTime = elapsedTime;

            turret.setControl(m_request.withPosition((calculateAngleToHub(velocityX, velocityY, robotPos))));
            limelightTurret = false;

            turretTARGET = (calculateAngleToHub(velocityX, velocityY, robotPos));

            System.out.println("using limelight-tags!");
        }
    }

    public double calculateTurretLimelightAngle() {
        return MathUtil.clamp(turret.getPosition().getValueAsDouble() + -txTurret,
                turret.getPosition().getValueAsDouble() - 10, turret.getPosition().getValueAsDouble() + 10);
    }

    public double calculateTurretPID(double velocityX, double velocityY, Translation2d robotpose) {

        double Hx = isBlue ? blueHubPos.getX() : redHubPos.getX();

        double hubOffsetX = Hx + (velocityX * ShooterSubsystem.tof);
        double hubOffsetY = redHubPos.getY() + (velocityY * ShooterSubsystem.tof);
        double originalHubAngle;
        double turretTxOffset;

        double offsetInDegrees = Math
                .toDegrees(Math.atan((hubOffsetY - robotpose.getY()) / (hubOffsetX - robotpose.getX())));

        double diffY = (redHubPos.getY() - robotPos.getY());
        double diffX = (Hx - robotPos.getX());
        originalHubAngle = Math.toDegrees(Math.atan2(diffY, diffX));

        turretTxOffset = originalHubAngle - offsetInDegrees;



        SmartDashboard.putNumber("limelight Turret Calculated ofset", offsetInDegrees);
        SmartDashboard.putNumber("turret tx offset", turretTxOffset);
        double pidPower;

        double kP = SmartDashboard.getNumber("turret kP", 0.0045);

        turretPID = new PIDController(kP, 0, 0);

        pidPower = MathUtil.clamp(turretPID.calculate(txTurret, 0), -0.2, 0.2);
        SmartDashboard.putNumber("pidPower", pidPower);
        SmartDashboard.putNumber("turret kP", kP);

        return pidPower;
    }

    public void setToZero() {
        turret.setControl(m_request.withPosition(-(0)));
    }

    public void determineLockingTarget(double velocityX, double velocityY) {

        double Hx = isBlue ? blueHubPos.getX() : redHubPos.getX();

        if (robotPos.getX() > blueHubPos.getX()) {
            feedingTargets();
        } else {
            lockingTarget = new Translation2d(Hx + (velocityX * ShooterSubsystem.tof),
                    redHubPos.getY() + (velocityY * ShooterSubsystem.tof));
            System.out.println("is not feeding!");
            isFeeding = false;
        }
        SmartDashboard.putBoolean("in deadzone?", turretDeadZone());
    }

    public void feedingTargets() {
        double tX = isBlue ? blueFx : redFx;
        double tYFar = robotPos.getY() > Hy && robotPos.getY() < 6.5 ? leftFyFar : rightFyFar;
        double tY = robotPos.getY() > Hy ? leftFy : rightFy;

        if (turretDeadZone()) {
            // do the deadzone swap!
            lockingTarget = new Translation2d(tX, tYFar);
            isFeeding = true;
        } else {
            lockingTarget = new Translation2d(tX, tY);
            isFeeding = true;
        }
    }

    public boolean turretDeadZone() {
        if (robotPos.getY() > 2.8 && robotPos.getY() < 5.3) {
            if (robotPos.getX() > 5.5 && robotPos.getX() < 7.0) {
                return true;
            } else {
                return false;
            }

        } else {
            return false;
        }
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
            LimelightHelpers.SetFidcuial3DOffset("limelight-turret", rotatedX, rotatedY, 0);
        }
    }

    public double calculateAngleToHub(double velocityX, double velocityY, Translation2d turretPose) {

        determineLockingTarget(0, 0);

        double diffY = (lockingTarget.getY() - turretPose.getY());
        double diffX = (lockingTarget.getX() - turretPose.getX());
        turretHubAngle = Math.toDegrees(Math.atan2(diffY, diffX));
        double goldenAngle = MathUtil.clamp(MathUtil.inputModulus((turretHubAngle - theta), -30, 330), -20, 315);

        return goldenAngle;

    }
}