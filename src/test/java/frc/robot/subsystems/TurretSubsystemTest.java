// package frc.robot.subsystems;

// import static org.junit.jupiter.api.Assertions.assertEquals;

// import org.junit.jupiter.api.Test;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;

// public class TurretSubsystemTest {

//     static TurretSubsystem subsystem = new TurretSubsystem(() -> new Pose2d());

//     @Test
//     public void testCalculateAngleToHub_inLineWithY() {
//         GameManager.isBlueAlliance = true;
//         double result = subsystem.calculateAngleToHub(0, 0, 
//             new Pose2d(subsystem.blueHubPos, Rotation2d.fromDegrees(0)));

//         assertEquals(subsystem.blueHubPos ,subsystem.lockingTarget);
//         assertEquals(0, result);
//     }

//     @Test
//     public void testCalculateAngleToHub_45degrees() {
//         GameManager.isBlueAlliance = true;
//         double result = subsystem.calculateAngleToHub(0, 0, 
//             new Pose2d(subsystem.blueHubPos.minus(new Translation2d(2, 2)), Rotation2d.fromDegrees(0)));
//         assertEquals(45, result);

//         result = subsystem.calculateAngleToHub(0, 0, 
//             new Pose2d(subsystem.blueHubPos.plus(new Translation2d(-2, 2)), Rotation2d.fromDegrees(0)));
//         assertEquals(360-45, result);
//     }

// }
