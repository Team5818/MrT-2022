package org.rivierarobotics.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.rivierarobotics.lib.MathUtil;

public class SwerveUtil {
    public static double clampAngle(double angle) {
        double high = Math.PI;
        angle = MathUtil.wrapToCircle(angle, 2 * Math.PI);
        if (angle > high) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static double getAngleDiff(double src, double target) {
        double diff = target - src;
        if (Math.abs(diff) <= Math.PI) {
            return diff;
        }

        if (diff > 0) {
            diff -= 2 * Math.PI;
        } else {
            diff += 2 * Math.PI;
        }

        return diff;
    }

    public static double getClosestAngle(double clampedAngle, double target) {
        double diff;
        double positiveTarget = target + Math.PI;
        double negativeTarget = target - Math.PI;
        if (Math.abs(getAngleDiff(clampedAngle, positiveTarget)) <= Math.abs(getAngleDiff(clampedAngle, negativeTarget))) {
            diff = getAngleDiff(clampedAngle, positiveTarget);
        } else {
            diff = getAngleDiff(clampedAngle, negativeTarget);
        }

        if (Math.abs(getAngleDiff(clampedAngle, target)) <= Math.abs(diff)) {
            diff = getAngleDiff(clampedAngle, target);
        }

        return diff;
    }

    public static SwerveModuleState optimizeSwerveStates(SwerveModuleState state, double moduleAngle) {
        double clampedAng = clampAngle(moduleAngle);
        double targetRotation = state.angle.getRadians();
        double diff = getClosestAngle(clampedAng, targetRotation);
        double targetAng = moduleAngle + diff;

        double targetSpeed = state.speedMetersPerSecond;
        if (!MathUtil.isWithinTolerance(targetRotation, clampAngle(targetAng), 0.1)) {
            targetSpeed *= -1;
        }

        return new SwerveModuleState(targetSpeed, new Rotation2d(targetAng));
    }
}
