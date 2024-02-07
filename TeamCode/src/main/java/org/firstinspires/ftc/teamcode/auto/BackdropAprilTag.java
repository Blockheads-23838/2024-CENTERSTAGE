package org.firstinspires.ftc.teamcode.auto;

// Locate a specific AprilTag and drive the robot into position
// in front of it, ready to deliver a yellow pixel.
import static org.firstinspires.ftc.teamcode.auto.vision.AprilTagUtils.AprilTagId.getEnumValue;
import org.firstinspires.ftc.teamcode.common.RobotContainer;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.vision.AprilTagUtils;
import org.firstinspires.ftc.teamcode.auto.vision.CameraToCenterCorrections;
import org.firstinspires.ftc.teamcode.auto.vision.AngleDistance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class BackdropAprilTag {
    private static final String TAG = BackdropAprilTag.class.getSimpleName();

    public enum Direction {
        FORWARD, BACK
    }

    private final LinearOpMode linearOpMode;

    // From the FTC sample ConceptAprilTag
    private AprilTagProcessor aprilTag;

    // As seen from behind.
    private static final double DISTANCE_CAMERA_LENS_TO_ROBOT_CENTER = 9.5; // LCHS 23838
    private static final double OFFSET_CAMERA_LENS_FROM_ROBOT_CENTER = -6.125; // LCHS 23838

    //**TODO Tune the next four values later - after the basic movement is working ...
    private static final double STRAFE_ADJUSTMENT_PERCENT = 0.0; // LCHS 23838
    private static final double OUTSIDE_STRAFE_ADJUSTMENT = 0.0; // LCHS 23838
    private static final double YELLOW_PIXEL_ADJUSTMENT = 0.0; // LCHS 23838
    private static final double DISTANCE_ADJUSTMENT_PERCENT = 0.0; // LCHS 23838

    //**TODO Instantiate this class in the init section of your Autonomous.
    public BackdropAprilTag(LinearOpMode pLinear) {
        linearOpMode = pLinear;
        initAprilTag();
    }

    // Assume that the robot's camera is parallel to the backdrop and that the
    // target AprilTag is in view.
    //**TODO Look at the test TeleOp OpMode FindBackdropAprilTag; it contains an
    // example of how to call this method.
    public boolean driveToBackdropAprilTag(AprilTagUtils.AprilTagId pTargetTagId, double pDesiredDistanceFromTag, Direction pDirection, RobotContainer container) {
        // First look for the target AprilTag on the backdrop.
        AprilTagDetectionData detectionData = findBackdropAprilTag(pTargetTagId);
        if (detectionData.ftcDetectionData == null) {
            return false; // no sure path to the backdrop
        }

        // If the backstop AprilTag that was found is not our target tag
        // then infer the position of the target tag.
        double aprilTagAngle;
        double aprilTagDistance;
        if (detectionData.aprilTagId != pTargetTagId) {
            RobotLog.dd(TAG, "Did not detect the target AprilTag " + pTargetTagId);
            RobotLog.dd(TAG, "Inferring its position from tag " + detectionData.aprilTagId);
            AngleDistance inferredPosition = AprilTagUtils.inferAprilTag(pTargetTagId, detectionData.aprilTagId,
                    detectionData.ftcDetectionData.ftcPose.range, detectionData.ftcDetectionData.ftcPose.bearing);
            aprilTagAngle = inferredPosition.angle;
            aprilTagDistance = inferredPosition.distance;
            RobotLog.dd(TAG, "Inferred distance " + aprilTagDistance + ", angle " + aprilTagAngle);
        } else {
            aprilTagDistance = detectionData.ftcDetectionData.ftcPose.range;
            aprilTagAngle = detectionData.ftcDetectionData.ftcPose.bearing;
            RobotLog.dd(TAG, "Using target AprilTag distance " + aprilTagDistance + ", angle " + aprilTagAngle);
        }

        // WSe assume the robot is square to the backdrop (the yaw is close to 0)
        // but let's see what the AprilTag detector thinks it is.
        RobotLog.dd(TAG, "Yaw as reported by the AprilTag detector " + detectionData.ftcDetectionData.ftcPose.yaw);

        RobotLog.dd(TAG, "Driving to AprilTag with id " + pTargetTagId);
        RobotLog.dd(TAG, "Stop at " + pDesiredDistanceFromTag + " from the tag");
        RobotLog.dd(TAG, "Direction of travel " + pDirection);

        // Unlike the RobotAutoDriveToAprilTagOmni sample, which tracks the
        // AprilTag in relation to the camera, we need the angle and distance
        // from the center of the robot, particularly if the camera is not
        // centered on the robot.

        // From the point of view of an observer facing the robot and the
        // backdrop from the center of the field -- a positive angle from
        // the camera to the AprilTag indicates that the tag is to the left
        // of the center of the robot (counter-clockwise).
        double angleFromRobotCenterToAprilTag =
                CameraToCenterCorrections.getCorrectedAngle(DISTANCE_CAMERA_LENS_TO_ROBOT_CENTER,
                        OFFSET_CAMERA_LENS_FROM_ROBOT_CENTER, aprilTagDistance, aprilTagAngle);

        double distanceFromRobotCenterToAprilTag =
                CameraToCenterCorrections.getCorrectedDistance(DISTANCE_CAMERA_LENS_TO_ROBOT_CENTER,
                        OFFSET_CAMERA_LENS_FROM_ROBOT_CENTER, aprilTagDistance, aprilTagAngle);

        RobotLog.dd(TAG, "Angle from robot center to AprilTag " + angleFromRobotCenterToAprilTag);
        RobotLog.dd(TAG, "Distance from robot center to AprilTag " + distanceFromRobotCenterToAprilTag);

        double distanceToMove;
        if (Math.abs(angleFromRobotCenterToAprilTag) >= 3.0) {
            // Strafe to place the center of the robot opposite the AprilTag.
            double sinTheta = Math.sin(Math.toRadians(Math.abs(angleFromRobotCenterToAprilTag)));
            double distanceToStrafe = Math.abs(sinTheta * distanceFromRobotCenterToAprilTag);
            double strafeVelocity = shortDistanceVelocity(distanceToStrafe);
            RobotLog.dd(TAG, "Calculated distance to strafe " + distanceToStrafe);

            // Add in strafe percentage adjustment.
            if (STRAFE_ADJUSTMENT_PERCENT != 0.0) {
                distanceToStrafe += (distanceToStrafe * STRAFE_ADJUSTMENT_PERCENT);
                RobotLog.dd(TAG, "Adjusting distance to strafe by a factor of " + STRAFE_ADJUSTMENT_PERCENT + " for a distance to strafe of " + distanceToStrafe);
            }

            // Call a method that adjusts the distance of the
            // strafe depending on the AprilTag. For both methods the parameter
            // "strafeDistanceFromAprilTagToRobotCenter" is the number of inches
            // that the center of the robot is left (positive) or right (negative)
            // of the AprilTag. The sign of the parameter is the same as the
            // inverse of the sign of the angle from the center of the robot to
            // the AprilTag.

            // The returned strafe angle (90.0 degrees or -90.0 degrees) is also
            // given from the point of view of an observer facing the backdrop.
            RobotLog.dd(TAG, "Including outside strafe adjustment of " + OUTSIDE_STRAFE_ADJUSTMENT);
            double signOfDistance = Math.signum(angleFromRobotCenterToAprilTag) * -1;
            AngleDistance adjustment =
                    AprilTagUtils.strafeAdjustment(pTargetTagId.getNumericId(), distanceToStrafe * signOfDistance, OUTSIDE_STRAFE_ADJUSTMENT, YELLOW_PIXEL_ADJUSTMENT);

            // Set the final angle to strafe with respect to the front of the
            // robot. The adjusted angle may have to be inverted depending on
            // whether the camera facing the backdrop is on the front (no
            // inversion) or back (inversion) of the robot.
            double directionFactor = (pDirection == Direction.FORWARD) ? 1.0 : -1.0;
            double strafeDirection = adjustment.angle * directionFactor;
            distanceToStrafe = adjustment.distance;
            RobotLog.dd(TAG, "Calculated final distance for strafe to yellow pixel delivery point " + distanceToStrafe);
            RobotLog.dd(TAG, "Strafe angle " + strafeDirection);

            // Check for a minimum distance to strafe.
            if (distanceToStrafe >= 1.0) {
                RobotLog.dd(TAG, "Strafe to yellow pixel delivery point " + distanceToStrafe);
                //**TODO Here's where you actually strafe your robot into position: the variable
                // strafeDirection is either 90.0 for a strafe to the left or -90.0 for a strafe
                // to the right.
                int targetClicks = (int) (distanceToStrafe * 50);
                container.goToVector(strafeDirection, targetClicks, strafeVelocity, true);
                // driveTrainMotion.straight(targetClicks, strafeDirection, strafeVelocity, 0, desiredHeading);
            }

            // Calculate the distance to move towards the backstop based on our triangle.
            // distanceFromRobotCenterToAprilTag (hypotenuse) squared = distanceToStrafe squared + adjacent squared.
            double adjacentSquared = Math.pow(distanceFromRobotCenterToAprilTag, 2) - Math.pow(distanceToStrafe, 2);
            double adjacent = Math.sqrt(adjacentSquared); // center of robot to AprilTag
            distanceToMove = adjacent - (DISTANCE_CAMERA_LENS_TO_ROBOT_CENTER + pDesiredDistanceFromTag);
            RobotLog.dd(TAG, "Adjusted pythagorean distance to move towards the backdrop " + distanceToMove);
        } else {
            distanceToMove = distanceFromRobotCenterToAprilTag - (DISTANCE_CAMERA_LENS_TO_ROBOT_CENTER + pDesiredDistanceFromTag);
            RobotLog.dd(TAG, "Calculated distance to move towards the backdrop " + distanceToMove);
        }

        // Move the robot towards the backstop. Take into account the robot's direction of travel.
        // Add in distance percentage adjustment.
        if (DISTANCE_ADJUSTMENT_PERCENT != 0.0) {
            distanceToMove += (distanceToMove * DISTANCE_ADJUSTMENT_PERCENT);
            RobotLog.dd(TAG, "Adjusting distance to move by " + DISTANCE_ADJUSTMENT_PERCENT);
        }

        double moveAngle = (pDirection == Direction.FORWARD) ? 0.0 : -180.0;
        double straightLineVelocity = .3;
        if (Math.abs(distanceToMove) >= 1.0) {
            RobotLog.dd(TAG, "Move robot towards the AprilTag " + distanceToMove + " inches");
            //**TODO Here's where you actually move your robot forward or backward into position:
            // the variable moveAngle is either 0.0 for forward movement or -180.0 for backward
            // movement.
            int targetClicks = (int) (distanceToMove * 50);
            container.goToVector(moveAngle, targetClicks, straightLineVelocity, true);
            // driveTrainMotion.straight(targetClicks, moveAngle, straightLineVelocity, 0, desiredHeading);
        }

        return true;
    }

    @SuppressLint("DefaultLocale")
    private AprilTagDetectionData findBackdropAprilTag(AprilTagUtils.AprilTagId pTargetTagId) {

        List<AprilTagDetection> currentDetections = aprilTag.getFreshDetections();
        AprilTagDetection targetDetection = null;
        AprilTagDetection backupDetection = null;
        double smallestBackupAngle = 360.0; // impossibly high

        // Step through the list of detected tags and look for a matching tag.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == pTargetTagId.getNumericId()) {
                    targetDetection = detection;
                    break; // don't look any further.
                } else {
                    if (Math.abs(detection.ftcPose.bearing) < smallestBackupAngle) {
                        smallestBackupAngle = detection.ftcPose.bearing;
                        backupDetection = detection;
                    }
                }
            }
        }

        // If we have found the target AprilTag return it now.
        if (targetDetection != null) {
            String targetTagString = "Found target AprilTag " + String.format("Id %d (%s)", targetDetection.id, targetDetection.metadata.name);
            String range = "Range " + String.format("%5.1f inches", targetDetection.ftcPose.range);
            String bearing = "Bearing " + String.format("%3.0f degrees", targetDetection.ftcPose.bearing);
            String yaw = "Yaw " + String.format("%3.0f degrees", targetDetection.ftcPose.yaw);

            linearOpMode.telemetry.addLine(targetTagString);
            linearOpMode.telemetry.update();

            RobotLog.dd(TAG, targetTagString);
            RobotLog.dd(TAG, range);
            RobotLog.dd(TAG, bearing);
            RobotLog.dd(TAG, yaw);
            return new AprilTagDetectionData(pTargetTagId, targetDetection);
        }

        // If we have not found the target target, see if we've found one of
        // the other AprilTags to use as a backup.
        if (backupDetection == null) {
            linearOpMode.telemetry.addLine("No AprilTags found within " + 2000 + "ms");
            linearOpMode.telemetry.update();
            RobotLog.dd(TAG, "No AprilTags found within " + 2000 + "ms");
            return new AprilTagDetectionData(pTargetTagId, null);
        }

        // Found a backup detection.
        String backupTagString = "Found backup AprilTag " + String.format("Id %d (%s)", backupDetection.id, backupDetection.metadata.name);
        String range = "Range " + String.format("%5.1f inches", backupDetection.ftcPose.range);
        String bearing = "Bearing " + String.format("%3.0f degrees", backupDetection.ftcPose.bearing);
        String yaw = "Yaw " + String.format("%3.0f degrees", backupDetection.ftcPose.yaw);

        linearOpMode.telemetry.addLine(backupTagString);
        linearOpMode.telemetry.update();

        RobotLog.dd(TAG, backupTagString);
        RobotLog.dd(TAG, range);
        RobotLog.dd(TAG, bearing);
        RobotLog.dd(TAG, yaw);
        return new AprilTagDetectionData(getEnumValue(backupDetection.id), backupDetection);
    }

    private static class AprilTagDetectionData {
        public final AprilTagUtils.AprilTagId aprilTagId;
        public final AprilTagDetection ftcDetectionData;

        public AprilTagDetectionData(AprilTagUtils.AprilTagId pAprilTagId, AprilTagDetection pFtcDetectionData) {
            aprilTagId = pAprilTagId;
            ftcDetectionData = pFtcDetectionData;
        }
    }

    // From ConceptAprilTag

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        //##PY in the sample all are commented out ...
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //##PY .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                // ... these parameters are fx, fy, cx, cy.
                /*
                StreamCam (home)
                    <focal_length_x>622.001</focal_length_x>
                    <focal_length_y>622.001</focal_length_y>
                    <principal_point_x>319.803</principal_point_x>
                    <principal_point_y>241.251</principal_point_y>
                 */
                //.setLensIntrinsics(622.001f, 622.001f, 319.803f, 241.251f)
                // Logitech C270 from FTC teamwebcamcalibrations.xml
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(linearOpMode.hardwareMap.get(WebcamName.class, "camera"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        builder.enableLiveView(false); //##PY - added

        // Build the Vision Portal, using the above settings.
        VisionPortal visionPortal = builder.build();

        RobotLog.d("ConceptAprilTag", "Waiting for webcam to start streaming");
        ElapsedTime streamingTimer = new ElapsedTime();
        streamingTimer.reset(); // start
        while (streamingTimer.milliseconds() < 2000 && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            linearOpMode.sleep(50);
        }

        VisionPortal.CameraState cameraState = visionPortal.getCameraState();
        if (cameraState != VisionPortal.CameraState.STREAMING) {
            throw new RuntimeException("Timed out waiting for webcam streaming to start");
        }
    }   // end method initAprilTag()

    //## We noticed that the robot took time starting and stopping for
    // short distances (such as 1.0") at .3 velocity while at longer
    // distances we sometimes want to keep the low velocity.
    private double shortDistanceVelocity(double pDistance) {
        return Math.abs(pDistance) < 2.0 ? 0.5 : 0.3;
    }

}