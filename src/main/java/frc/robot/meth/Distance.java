package frc.robot.meth;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.meth.Alliance.apply;

public class Distance {

    /**
     * Checks if a point is within a specified distance from a line segment
     *
     * @param point       The point to check
     * @param lineStart   Start point of the line segment
     * @param lineEnd     End point of the line segment
     * @param maxDistance The maximum allowed distance from the line segment
     * @return true if the point is within maxDistance from the line segment
     */
    public static boolean isPointNearLineSegment(Translation2d point, Pose2d center, double lineLength, double maxDistance) {
        Translation2d[] translation2ds = generateLinePoints(center, lineLength);
        Translation2d lineStart = translation2ds[0];
        Translation2d lineEnd = translation2ds[1];
        // Calculate vectors
        Translation2d lineVec = lineEnd.minus(lineStart);
        Translation2d pointVec = point.minus(lineStart);

        // Get squared length of line segment
        double lineLengthSquared = lineVec.getX() * lineVec.getX() + lineVec.getY() * lineVec.getY();

        // If line segment has zero length, just check distance to start point
        if (lineLengthSquared == 0) {
            return point.getDistance(lineStart) <= maxDistance;
        }

        // Calculate projection factor
        double t = Math.max(0, Math.min(1,
                (pointVec.getX() * lineVec.getX() + pointVec.getY() * lineVec.getY()) / lineLengthSquared
        ));

        // Calculate closest point on line segment
        Translation2d projection = new Translation2d(
                lineStart.getX() + t * lineVec.getX(),
                lineStart.getY() + t * lineVec.getY()
        );

        // Check if distance to closest point is within maxDistance
        return point.getDistance(projection) <= maxDistance;
    }

    /**
     * Generates start and end points for a line segment given a center point, angle, and length.
     * Properly mirrors coordinates across field centerline if on red alliance.
     *
     * @param center The center point of the line segment
     * @param length The total length of the line segment
     * @return An array of two Translation2d points: [startPoint, endPoint]
     */
    public static Translation2d[] generateLinePoints(Pose2d center, double length) {
        // Convert angle to Rotation2d
        Rotation2d angle = center.getRotation();

        // Calculate the offset from center to either endpoint
        double halfLength = length / 2.0;
        Translation2d offset = new Translation2d(
                halfLength * angle.getCos(),
                halfLength * angle.getSin()
        );

        // Generate the initial points
        Translation2d startPoint = center.getTranslation().minus(offset);
        Translation2d endPoint = center.getTranslation().plus(offset);

        // Apply alliance flipping if needed
        startPoint = apply(startPoint);
        endPoint = apply(endPoint);


        return new Translation2d[]{startPoint, endPoint};
    }
}
