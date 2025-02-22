import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.meth.Distance;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

class DistanceTest {

    private static final double EPSILON = 1e-6;

    @Test
    void testIsPointNearLineSegment_OnLine() {
        Pose2d center = new Pose2d(new Translation2d(5, 5), new Rotation2d(0));
        double lineLength = 4.0;
        double maxDistance = 1.0;
        Translation2d point = new Translation2d(6, 5); // Point on the line

        Assertions.assertTrue(Distance.isPointNearLineSegment(point, center, lineLength, maxDistance));
    }

    @Test
    void testIsPointNearLineSegment_WithinDistance() {
        Pose2d center = new Pose2d(new Translation2d(5, 5), new Rotation2d(0));
        double lineLength = 4.0;
        double maxDistance = 1.5;
        Translation2d point = new Translation2d(6, 6); // 1 unit away from the line

        Assertions.assertTrue(Distance.isPointNearLineSegment(point, center, lineLength, maxDistance));
    }

    @Test
    void testIsPointNearLineSegment_OutsideDistance() {
        Pose2d center = new Pose2d(new Translation2d(5, 5), new Rotation2d(0));
        double lineLength = 4.0;
        double maxDistance = 1.0;
        Translation2d point = new Translation2d(6, 7); // More than 1 unit away

        Assertions.assertFalse(Distance.isPointNearLineSegment(point, center, lineLength, maxDistance));
    }

    @Test
    void testDistanceOfPointNearLineSegment() {
        Pose2d center = new Pose2d(new Translation2d(5, 5), new Rotation2d(0));
        double lineLength = 4.0;
        Translation2d point = new Translation2d(6, 6); // 1 unit away from the line

        double distance = Distance.distanceOfPointNearLineSegment(point, center, lineLength);
        Assertions.assertEquals(1.0, distance, EPSILON);
    }

    @Test
    void testDistanceOfPointNearLineSegment_ZeroLength() {
        Pose2d center = new Pose2d(new Translation2d(5, 5), new Rotation2d(0));
        double lineLength = 0.0;
        Translation2d point = new Translation2d(6, 6);

        double distance = Distance.distanceOfPointNearLineSegment(point, center, lineLength);
        Assertions.assertEquals(Math.sqrt(2), distance, EPSILON);
    }

    @Test
    void testGenerateLinePoints() {
        Pose2d center = new Pose2d(new Translation2d(5, 5), new Rotation2d(0));
        double lineLength = 4.0;

        Translation2d[] points = Distance.generateLinePoints(center, lineLength);
        Translation2d expectedStart = new Translation2d(3, 5);
        Translation2d expectedEnd = new Translation2d(7, 5);

        Assertions.assertEquals(expectedStart.getX(), points[0].getX(), EPSILON);
        Assertions.assertEquals(expectedStart.getY(), points[0].getY(), EPSILON);
        Assertions.assertEquals(expectedEnd.getX(), points[1].getX(), EPSILON);
        Assertions.assertEquals(expectedEnd.getY(), points[1].getY(), EPSILON);
    }

    @Test
    void testIsPointNearLinesSegment_FindsClosest() {
        Pose2d[] centers = {
                new Pose2d(new Translation2d(5, 5), new Rotation2d(0)),
                new Pose2d(new Translation2d(10, 10), new Rotation2d(Math.PI / 4))
        };
        double lineLength = 4.0;
        double maxDistance = 2.0;
        Translation2d point = new Translation2d(6, 5); // Closer to first line

        Pose2d closest = Distance.isPointNearLinesSegment(point, centers, lineLength, maxDistance);
        Assertions.assertNotNull(closest);
        Assertions.assertEquals(5, closest.getX(), EPSILON);
        Assertions.assertEquals(5, closest.getY(), EPSILON);
    }

    @Test
    void testIsPointNearLinesSegment_NoCloseLines() {
        Pose2d[] centers = {
                new Pose2d(new Translation2d(5, 5), new Rotation2d(0)),
                new Pose2d(new Translation2d(10, 10), new Rotation2d(Math.PI / 4))
        };
        double lineLength = 4.0;
        double maxDistance = 1.0;
        Translation2d point = new Translation2d(20, 20); // Too far away

        Pose2d closest = Distance.isPointNearLinesSegment(point, centers, lineLength, maxDistance);
        Assertions.assertNull(closest);
    }
}
