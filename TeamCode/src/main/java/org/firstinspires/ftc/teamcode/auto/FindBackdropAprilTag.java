package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.vision.AprilTagUtils;
import org.firstinspires.ftc.teamcode.common.RobotContainer;

@TeleOp(name = "Find Backdrop AprilTag", group = "Test")
//@Disabled
public class FindBackdropAprilTag extends LinearOpMode {
    RobotContainer robotContainer = null;
    @Override
    public void runOpMode() {
        BackdropAprilTag backdropAprilTag = new BackdropAprilTag(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        //**TODO Assuming that the output of your Team Prop recognition is an integer
        // from 1 to 6, first convert it to an enumeration value for the AprilTags:
        AprilTagUtils.AprilTagId aprilTagEnum = AprilTagUtils.AprilTagId.getEnumValue(2);

        //**TODO Then call the following method to position your robot in front of
        // the identified AprilTag for delivery of the yellow pixel. The third
        // parameter will always be the same - in your case the camera is on the
        // front of your robot.
        // backdropAprilTag.driveToBackdropAprilTag(aprilTagEnum,
        //         2.0, BackdropAprilTag.Direction.FORWARD);
    }

}