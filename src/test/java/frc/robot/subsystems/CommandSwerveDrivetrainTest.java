// package frc.robot.subsystems;

// import static org.junit.jupiter.api.Assertions.assertEquals;

// import org.junit.jupiter.api.Test;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.SwerveConstants;

// public class CommandSwerveDrivetrainTest {

//     static CommandSwerveDrivetrain subsystem = SwerveConstants.createDrivetrain();

//     @Test
//     public void testGetTurretOffet_transformationCorrectlyAccountsForTurretOffset() {
//         GameManager.isBlueAlliance = true;
//         subsystem.resetPose(new Pose2d());
//         Pose2d result = subsystem.getTurretOffset();
//         assertEquals(new Pose2d(0.1434465, 0.0746125, Rotation2d.fromDegrees(0)), result);

//         subsystem.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
//         result = subsystem.getTurretOffset();
//         assertEquals(new Pose2d(-0.1434465, -0.0746125, Rotation2d.fromDegrees(180)), result);

//         subsystem.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
//         result = subsystem.getTurretOffset();
//         assertEquals(new Pose2d(-0.0746125, 0.1434465, Rotation2d.fromDegrees(90)), result);

//         subsystem.resetPose(new Pose2d(2, 1, Rotation2d.fromDegrees(90)));
//         result = subsystem.getTurretOffset();
//         assertEquals(new Pose2d(2-0.0746125, 1+0.1434465, Rotation2d.fromDegrees(90)), result);
//     }

// }
