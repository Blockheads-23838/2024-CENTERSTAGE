package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

import java.util.EnumSet;

public class AprilTagUtils {

    private static final String TAG = AprilTagUtils.class.getSimpleName();

    // AprilTag identifiers
    public enum AprilTagId {
        TAG_ID_NPOS(-1),
        TAG_ID_1(1), TAG_ID_2(2), TAG_ID_3(3),
        TAG_ID_4(4), TAG_ID_5(5), TAG_ID_6(6),
        TAG_ID_7(7), TAG_ID_8(8), TAG_ID_9(9),
        TAG_ID_10(10);

        private final int numericAprilTagId;

        AprilTagId(int pNumericId) {
            numericAprilTagId = pNumericId;
        }

        public int getNumericId() {
            return numericAprilTagId;
        }

        // Given the numeric id of an AprilTag return its
        // enumeration.
        public static AprilTagUtils.AprilTagId getEnumValue(int pNumericId) {
            AprilTagUtils.AprilTagId[] tagValues = AprilTagUtils.AprilTagId.values();
            for (AprilTagUtils.AprilTagId tagValue : tagValues) {
                if (tagValue.numericAprilTagId == pNumericId)
                    return tagValue;
            }

            return AprilTagUtils.AprilTagId.TAG_ID_NPOS; // no match
        }
    }

    private static final double STRAFE_LEFT = 90.0;
    private static final double STRAFE_RIGHT = -90.0;

    // For validation of the AprilTags on the BLUE side backdrop.
    private static final EnumSet<AprilTagUtils.AprilTagId> blueBackdropAprilTags =
            EnumSet.of(AprilTagUtils.AprilTagId.TAG_ID_1,
                    AprilTagUtils.AprilTagId.TAG_ID_2,
                    AprilTagUtils.AprilTagId.TAG_ID_3);

    // For validation of the AprilTags on the RED side backdrop.
    private static final EnumSet<AprilTagUtils.AprilTagId> redBackdropAprilTags =
            EnumSet.of(AprilTagUtils.AprilTagId.TAG_ID_4,
                    AprilTagUtils.AprilTagId.TAG_ID_5,
                    AprilTagUtils.AprilTagId.TAG_ID_6);

    // Call this method if the target AprilTag on the backstop could not
    // be located but at least one if its near neighbors in the same set
    // of three could be located. This method infers the location of the
    // target AprilTag from the position of its neighbor and returns the
    // distance and angle to the target AprilTag.

    public static AngleDistance inferAprilTag(AprilTagUtils.AprilTagId pTargetId,
                                              AprilTagUtils.AprilTagId pRecognizedId,
                                              double pDistanceToRecognizedId, double pAngleToRecognizedId) {
        if (pTargetId == pRecognizedId) // the caller shouldn't do this
            return new AngleDistance(pAngleToRecognizedId, pDistanceToRecognizedId); // so return the input

        // Validate that the target id and the recognized id belong to the
        // backdrop of the same alliance.
        if (!((blueBackdropAprilTags.contains(pTargetId) && blueBackdropAprilTags.contains(pRecognizedId)) ||
                (redBackdropAprilTags.contains(pTargetId) && redBackdropAprilTags.contains(pRecognizedId))))
            throw new RuntimeException("Target and recognized AprilTag ids do not belong to the same alliance");

        return calculator(pTargetId.getNumericId(), pRecognizedId.getNumericId(),
                pDistanceToRecognizedId, pAngleToRecognizedId);
    }

    // Returns the inferred angle and inferred distance to the target AprilTag.
    // Credit to Notre Dame students Brandon Lim for the calculations and Sotiris
    // Artenos for the IntelliJ test harness.
    private static AngleDistance calculator(int targetAprilTag, int givenAprilTag, double distanceGivenApril, double angleGivenApril) {

        int sign = targetAprilTag < givenAprilTag ? -1 : 1;
        int endsign = targetAprilTag > givenAprilTag ? -1 : 1;

        //Math shown in the notebook

        angleGivenApril = sign * Math.toRadians(angleGivenApril);
        double c = distanceGivenApril * Math.sin(angleGivenApril);
        double f = (Math.abs(targetAprilTag - givenAprilTag) * 6) - c;
        double d = distanceGivenApril * Math.cos(angleGivenApril);

        double newAngleApril = Math.atan(f / d);
        newAngleApril = endsign * Math.toDegrees(newAngleApril);
        double distanceFive = Math.sqrt(d * d + f * f);

        return new AngleDistance(newAngleApril, distanceFive);
    }

    // The method strafeAdjustment is used for positions F4 and A4.
    // It is used for ending on the right or left side of the backdrop.
    // It adjusts the amount of the strafe so that the robot can drop
    // the yellow pixel at the edge of the backdrop.

    // This method assumes that an observer is facing the robot and
    // the backdrop from the center of the field. The parameter
    // "strafeDistanceFromAprilTagToRobotCenter" is the number
    // of inches that the center of the robot is left (positive) or
    // right (negative) of the AprilTag.

    // The returned angle to strafe (90.0 degrees or -90.0 degrees)
    // assumes the same point of view. The returned distance is the
    // positive distance to strafe.
    public static AngleDistance strafeAdjustment(int aprilTag, double strafeDistanceFromAprilTagToRobotCenter, double outsideAdjustment, double yellowPixelAdjustment) {
        // for center april tags
        // no change is needed for center april tags
        if (aprilTag == 2 || aprilTag == 5) {
            if (strafeDistanceFromAprilTagToRobotCenter >= 0) {
                return new AngleDistance(STRAFE_RIGHT, strafeDistanceFromAprilTagToRobotCenter + yellowPixelAdjustment);
            } else {
                return new AngleDistance(STRAFE_LEFT, Math.abs(strafeDistanceFromAprilTagToRobotCenter) + yellowPixelAdjustment);
            }
        }

        // for left april tags
        else if (aprilTag == 1 || aprilTag == 4) {

            // left of april tag
            if (strafeDistanceFromAprilTagToRobotCenter >= 0) {
                // not far enough to the left from april tag
                // if robot is not far enough to the left the robot has to move a small amount more left
                if (strafeDistanceFromAprilTagToRobotCenter - outsideAdjustment < 0) {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(strafeDistanceFromAprilTagToRobotCenter - outsideAdjustment));
                }

                // too far to the left of april tag
                // if robot too far from april tag robot has to move right to be outsideAdjustment inches from april tag
                else {
                    return new AngleDistance(STRAFE_RIGHT, Math.abs(strafeDistanceFromAprilTagToRobotCenter - outsideAdjustment));
                }
            }

            // right of april tag
            // has to move more left to reach outsideAdjustment inches from april tag
            else {
                return new AngleDistance(STRAFE_LEFT, Math.abs(Math.abs(strafeDistanceFromAprilTagToRobotCenter) + outsideAdjustment));
            }
        }

        // for right april tags
        else {

            // left of april tag
            // has to move more right to reach outsideAdjustment inches from april tag
            if (strafeDistanceFromAprilTagToRobotCenter >= 0) {
                return new AngleDistance(STRAFE_RIGHT, Math.abs(Math.abs(strafeDistanceFromAprilTagToRobotCenter) + outsideAdjustment));
            }

            // right of april tag
            else {

                // too far to the left of april tag
                // if robot too far from april tag robot has to move right to be outsideAdjustment inches from april tag
                if (strafeDistanceFromAprilTagToRobotCenter + outsideAdjustment < 0) {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(strafeDistanceFromAprilTagToRobotCenter + outsideAdjustment));
                }

                // not far enough to the left from april tag
                // if robot is not far enough to the right the robot has to move a small amount more right
                else {
                    return new AngleDistance(STRAFE_RIGHT, Math.abs(strafeDistanceFromAprilTagToRobotCenter + outsideAdjustment));
                }
            }
        }
    }

    // The method yellowPixelAdjustment is used for positions F2 and A2.
    // It is used in order to move the robot a small distance left or right
    // in order to place the yellow pixel in the other slot on the backdrop.
    public static AngleDistance yellowPixelAdjustment(int aprilTag, double strafeDistanceFromAprilTagToRobotCenter, RobotConstantsCenterStage.BackdropPixelOpenSlot openSlot, double yellowPixelAdjustment, double outsideAdjustment) {
        if (strafeDistanceFromAprilTagToRobotCenter >= 0) {
            switch (openSlot) {
                case LEFT: {
                    return new AngleDistance(STRAFE_RIGHT, Math.abs(strafeDistanceFromAprilTagToRobotCenter - yellowPixelAdjustment));
                }
                case RIGHT: {
                    return new AngleDistance(STRAFE_RIGHT, Math.abs(strafeDistanceFromAprilTagToRobotCenter + yellowPixelAdjustment));
                }
                case ANY_OPEN_SLOT: {
                    return strafeAdjustment(aprilTag, strafeDistanceFromAprilTagToRobotCenter, outsideAdjustment, yellowPixelAdjustment);
                }
                default: {
                    return new AngleDistance(STRAFE_RIGHT, strafeDistanceFromAprilTagToRobotCenter);
                }
            }
        } else {
            switch (openSlot) {
                case LEFT: {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(strafeDistanceFromAprilTagToRobotCenter + yellowPixelAdjustment));
                }
                case RIGHT: {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(strafeDistanceFromAprilTagToRobotCenter - yellowPixelAdjustment));
                }
                case ANY_OPEN_SLOT: {
                    return strafeAdjustment(aprilTag, strafeDistanceFromAprilTagToRobotCenter, outsideAdjustment, yellowPixelAdjustment);
                }
                default: {
                    return new AngleDistance(STRAFE_LEFT, Math.abs(strafeDistanceFromAprilTagToRobotCenter));
                }
            }
        }
    }

}