package org.firstinspires.ftc.teamcode.auto.vision;

/*
Contributed by Christian Giron-Michel
Notre Dame High School
November 2022
 */
public class CameraToCenterCorrections {

    public static final double pi = Math.PI;

    //The formulas change depending on whether we have the camera positioned to the left or to the right of the center of the robot.
    public enum CameraPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    //Computes the distance from the center of the robot to the object using a formula derived from the law of cosines.
    public static double getCorrectedDistance(double distanceFromCenterToFront, double offset, double distanceFromCamera, double angleFromCamera){

        //Converts the input angle from degrees to radians.
        angleFromCamera = Math.toRadians(angleFromCamera);

        //Returns the direction of the camera offset from the center of the robot when looking from behind.
        CameraPosition cameraPosition;
        if (offset < 0){
            cameraPosition = CameraPosition.RIGHT;
        }
        else if (offset > 0){
            cameraPosition = CameraPosition.LEFT;
        }
        else {
            cameraPosition = CameraPosition.CENTER;
        }

        //Gets the magnitude of the horizontal offset to ensure that angles stay positive.
        offset = Math.abs(offset);

        //Measure in inches the distance from the camera to the center of the robot.
        //Always positive.
        double distanceFromCenterToCamera = Math.sqrt(Math.pow(distanceFromCenterToFront, 2) + Math.pow(offset, 2));

        //Measure in degrees from the heading of the camera to a line that connects the camera & the center of the robot.
        //Always positive & less than 180 degrees.
        double angleFromCameraToCenter = Math.toRadians(90) + Math.atan(distanceFromCenterToFront/offset);

        //The sum of the squares of the distance between the center of the robot & the camera and the distance between the camera & the object.
        double pythSum = Math.pow(distanceFromCenterToCamera, 2) + Math.pow(distanceFromCamera, 2);

        switch (cameraPosition) {
            case LEFT: {
                //Returns the distance from the center of the robot to the object.
                return Math.sqrt(pythSum - 2 * distanceFromCamera * distanceFromCenterToCamera * Math.cos(angleFromCameraToCenter + angleFromCamera));
            }
            case CENTER: {
                //Returns the distance from the center of the robot to the object.
                return Math.sqrt(pythSum - 2 * distanceFromCamera * distanceFromCenterToCamera * Math.cos(pi - angleFromCamera));
            }
            case RIGHT: {
                //Returns the distance from the center of the robot to the object.
                return Math.sqrt(pythSum - 2 * distanceFromCamera * distanceFromCenterToCamera * Math.cos(angleFromCameraToCenter - angleFromCamera));
            }
        }

        return 0;

    }

    //Computes the angle from the heading of the robot to the object using a formula derived from the law of sines.
    //Returns a positive angle if the robot needs to turn counterclockwise.
    //Returns a negative angle if the robot needs to turn clockwise.
    public static double getCorrectedAngle(double distanceFromCenterToFront, double offset, double distanceFromCamera, double angleFromCamera){

        //Converts the input angle from degrees to radians.
        angleFromCamera = Math.toRadians(angleFromCamera);

        //Returns the direction of the camera offset from the center of the robot when looking from behind.
        CameraPosition cameraPosition;
        if (offset < 0){
            cameraPosition = CameraPosition.RIGHT;
        }
        else if (offset > 0){
            cameraPosition = CameraPosition.LEFT;
        }
        else {
            cameraPosition = CameraPosition.CENTER;
        }

        //Gets the magnitude of the horizontal offset to ensure that angles stay positive.
        offset = Math.abs(offset);

        //Measure in degrees from the heading of the camera to a line that connects the camera & the center of the robot.
        //Always positive & less than 180 degrees.
        double angleFromCameraToCenter = Math.toRadians(90) + Math.atan(distanceFromCenterToFront/offset);
        //Measure in degrees from the heading of the center of the robot (the robot itself) to the line that connects the camera & the center of the robot.
        //Supplement to "angleFromCameraToCenter"
        double angleFromCenterToCamera = pi - angleFromCameraToCenter;

        switch (cameraPosition) {
            case LEFT: {
                //Computes the distance from the center of the robot to the object using the distance function defined above.
                double distanceFromCenter = getCorrectedDistance(distanceFromCenterToFront, offset, distanceFromCamera, Math.toDegrees(angleFromCamera));
                //Computes the sine of the angle between the line connecting the camera & the object and the line connecting the center of the robot to the camera.
                double sinOppositeCenterLine = Math.sin(angleFromCameraToCenter + angleFromCamera);
                //Computes the angle between the line connecting the center of the robot to the camera & the line connecting the center of the robot to the object.
                double theta = Math.asin((distanceFromCamera * sinOppositeCenterLine) / distanceFromCenter);
                //Returns the angle between the heading of the robot & the line connecting the center of the robot to the object.
                return Math.toDegrees(angleFromCenterToCamera - theta);

            }
            case CENTER: {
                //Computes the distance from the center of the robot to the object using the distance function defined above.
                double distanceFromCenter = getCorrectedDistance(distanceFromCenterToFront, offset, distanceFromCamera, Math.toDegrees(angleFromCamera));
                //Computes the sine of the angle between the line connecting the camera & the object and the line connecting the center of the robot to the camera.
                double sinOppositeCenterLine = Math.sin(pi - angleFromCamera);
                //Computes the angle between the line connecting the center of the robot to the camera & the line connecting the center of the robot to the object.
                double theta = Math.asin((distanceFromCamera * sinOppositeCenterLine)/distanceFromCenter);
                //Returns the angle between the heading of the robot & the line connecting the center of the robot to the object.
                return Math.toDegrees(theta);
            }
            case RIGHT: {
                //Computes the distance from the center of the robot to the object using the distance function defined above.
                double distanceFromCenter = getCorrectedDistance(distanceFromCenterToFront, offset, distanceFromCamera, Math.toDegrees(angleFromCamera));
                //Computes the sine of the angle between the line connecting the camera & the object and the line connecting the center of the robot to the camera.
                double sinOppositeCenterLine = Math.sin(angleFromCameraToCenter - angleFromCamera);
                //Computes the angle between the line connecting the center of the robot to the camera & the line connecting the center of the robot to the object.
                double theta = Math.asin((distanceFromCamera * sinOppositeCenterLine)/distanceFromCenter);
                //Returns the angle between the heading of the robot & the line connecting the center of the robot to the object.
                return Math.toDegrees(-(angleFromCenterToCamera - theta));
            }
        }

        return 0;
    }

}