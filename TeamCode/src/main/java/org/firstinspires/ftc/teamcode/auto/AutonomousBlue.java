package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "Blue auto")
public class AutonomousBlue extends LinearOpMode {

    private DcMotorEx lift;
    private Servo servo;
    private DcMotor intake = null;
    OpenCvCamera camera;
    BasicPipeline pipeline = new BasicPipeline();

    public static Pose2d toPropPose = new Pose2d(9, 36 - 6, Math.toRadians(180));
    public static double toPropPoseAngle = 225;
    public static Pose2d pixelDropPose = new Pose2d(12, 36 - 6, Math.toRadians(180));
    public static double pixelDropPoseAngle = 180;
    // private VisionPortal visionPortal;               // Used to manage the video source.
    // private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    @Override
    public void runOpMode() throws InterruptedException {
        // Camera activation
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Scoring activation
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        intake = hardwareMap.get(DcMotor.class, "intake");
        servo = hardwareMap.get(Servo.class, "servo");

        lift.setDirection(Constants.motorDirections.get("lift"));

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setVelocity(1);
        servo.setPosition(0.1);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Prop x value: ", pipeline.getJunctionPoint().x);
            telemetry.addData("Prop area: ", pipeline.getPropAreaAttr());
            telemetry.addLine("Place the purple pixel between the second and third compliant wheels from the left.");
            telemetry.addLine("It should be roughly centered.  It should be as close to touching the ground as possible WITHOUT touching the ground.");
            telemetry.addLine("Ensure the intake is at the bottom of its backlash-induced free-spinning zone so the pixel doesn't scrape the ground.");
            telemetry.update();
        }

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setVelocity(1);

        // Drivetrain activation
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startingPose = new Pose2d(15, 60+3, Math.toRadians(270));
        drive.setPoseEstimate(startingPose);


        TrajectorySequence rightTrajectory = drive.trajectorySequenceBuilder(startingPose)
                .splineToLinearHeading(new Pose2d(24, 39, Math.toRadians(180)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intake.setPower(0);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence middleTrajectory = drive.trajectorySequenceBuilder(startingPose)
                .splineToLinearHeading(new Pose2d(17, 40, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(20, 48, Math.toRadians(270)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intake.setPower(0);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence leftTrajectory = drive.trajectorySequenceBuilder(startingPose)
                .splineToLinearHeading(new Pose2d(14, 36, Math.toRadians(0)), Math.toRadians(225))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intake.setPower(0);
                })
                .waitSeconds(1)
                .build();


        double propX = pipeline.getJunctionPoint().x;
        double propArea = pipeline.getPropAreaAttr();

        if (propArea < 15000) { // None detected, we assume left spike mark
            drive.followTrajectorySequence(leftTrajectory);
        } else if (propX > 600) { // right spike mark
            drive.followTrajectorySequence(rightTrajectory);
        } else { // middle spike mark
            drive.followTrajectorySequence(middleTrajectory);

        }

        sleep(400);
        lift.setTargetPosition(1500);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setVelocity(1000);
        while (lift.isBusy()) telemetry.addData("lift position: ", lift.getCurrentPosition());
        // wiggle the pixel off
        for (int i = 0; i < 20; i++) {
            servo.setPosition(0.19);
            sleep(100);
            servo.setPosition(0.1);
            sleep(100);
        }
    }
}
