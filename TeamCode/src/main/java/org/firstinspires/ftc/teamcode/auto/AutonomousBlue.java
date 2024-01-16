package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.databind.introspect.TypeResolutionContext;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "Blue auto")
public class AutonomousBlue extends LinearOpMode {

    private DcMotorEx lift;
    private Servo servo;
    private DcMotor intake = null;
    DcMotorEx leftFrontDrive;
    DcMotorEx leftBackDrive;
    DcMotorEx rightFrontDrive;
    DcMotorEx rightBackDrive;

    ArrayList<DcMotorEx> driveMotors = new ArrayList<>();
    OpenCvCamera camera;
    BasicPipeline pipeline = new BasicPipeline();

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
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");

        leftFrontDrive.setDirection(Constants.motorDirections.get("left_front"));
        leftBackDrive.setDirection(Constants.motorDirections.get("left_back"));
        rightFrontDrive.setDirection(Constants.motorDirections.get("right_front"));
        rightBackDrive.setDirection(Constants.motorDirections.get("right_back"));

        driveMotors.add(leftFrontDrive);
        driveMotors.add(leftBackDrive);
        driveMotors.add(rightFrontDrive);
        driveMotors.add(rightBackDrive);

        double propX = pipeline.getJunctionPoint().x;
        double propArea = pipeline.getPropAreaAttr();

        double autoPower = 500;

        goTo(1175, 0, 0, autoPower, true);

        if (propArea < 15000) { // None detected, we assume left spike mark
            goTo(100, 0, 0, autoPower, true);
            goTo(0, 0, -90, autoPower, true);
        } else if (propX > 600) { // right spike mark
            goTo(0, 0, 90, autoPower, true);
        } else { // middle spike mark
            goTo(0, 300, 0, autoPower, true);
            // the forward already aligned us lololol
        }
        sleep(500);
        intake.setPower(-1);
        sleep(1000);
        intake.setPower(0);

    }
    public void goTo(double forward, double strafe, double yaw, double powercoef, boolean waitToFinish) {
        /**
         * Moves robot-centrically.  All is in ticks except yaw, which is approximately degrees such that 90 turns the robot 90 degrees clockwise.
         */
        yaw *= 11;
        double leftFrontPower = powercoef * (forward + strafe + yaw);
        double rightFrontPower = powercoef * (forward - strafe - yaw);
        double leftBackPower = powercoef * (forward - strafe + yaw);
        double rightBackPower = powercoef * (forward + strafe - yaw);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 2000) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
            leftFrontPower *= powercoef;
            leftBackPower *= powercoef;
            rightFrontPower *= powercoef;
            rightBackPower *= powercoef;
        }

        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        leftFrontDrive.setTargetPosition((int) (forward + strafe + yaw));
        leftBackDrive.setTargetPosition((int) (forward - strafe + yaw));
        rightFrontDrive.setTargetPosition((int) (forward - strafe - yaw));
        rightBackDrive.setTargetPosition((int) (forward + strafe - yaw));

        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftFrontDrive.setVelocity(leftFrontPower);
        leftBackDrive.setVelocity(leftBackPower);
        rightFrontDrive.setVelocity(rightFrontPower);
        rightBackDrive.setVelocity(rightBackPower);

        if (waitToFinish) while (leftFrontDrive.isBusy()) {
            telemetry.addData("lf power: ", leftFrontPower);
            telemetry.addData("lf tgt position: ", leftFrontDrive.getTargetPosition());
            telemetry.addData("lf position: ", leftFrontDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}
