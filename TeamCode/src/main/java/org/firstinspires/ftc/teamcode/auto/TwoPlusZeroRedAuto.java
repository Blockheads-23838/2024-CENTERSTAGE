package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "2+0 Red")
public class TwoPlusZeroRedAuto extends LinearOpMode {

    private DcMotorEx lift;
    private DcMotor intake = null;
    DcMotorEx leftFrontDrive;
    DcMotorEx leftBackDrive;
    DcMotorEx rightFrontDrive;
    DcMotorEx rightBackDrive;
    private Servo servo;

    ArrayList<DcMotorEx> driveMotors = new ArrayList<>();
    OpenCvCamera camera;
    PipelineRed pipeline = new PipelineRed();

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

        lift.setDirection(Constants.motorDirections.get("lift"));

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setVelocity(1);

        servo = hardwareMap.get(Servo.class, "servo");

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Prop x value: ", pipeline.getJunctionPoint().x);
            telemetry.addData("Prop area: ", pipeline.getPropAreaAttr());
            telemetry.addLine("Place the purple pixel between the second and third compliant wheels from the left.");
            telemetry.addLine("It should be roughly centered.  It should be as close to touching the ground as possible WITHOUT touching the ground.");
            telemetry.addLine("Ensure the intake is at the bottom of its backlash-induced free-spinning zone so the pixel doesn't scrape the ground.");
            telemetry.addLine("The pan should be FULLY ON THE GROUND when the program starts.");
            telemetry.update();
        }
        servo.setPosition(0.09);

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
            /*
            Testing 1/29/24 with yellow pixel in middle of pan
            I I I I i(setup issue :P) I
             */
            goTo(100, 0, 0, autoPower, true);
            goTo(0, 0, -90, autoPower, true);

            sleep(4);
            intake.setPower(-1);
            sleep(1000);
            intake.setPower(0);

            // Go to tile in front of backboard
            lift.setTargetPosition(1700);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(700);
            goTo(-1500, 400, -10, autoPower, true);
            while (lift.isBusy()) telemetry.addLine("Waiting for lift to reach setpoint...");

            // Go to backboard and score
            lift.setTargetPosition(2000);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(300);
            goTo(-200, 0, 0, autoPower, true);
            for (int i = 0; i < 1; i++) {
                servo.setPosition(0.20);
                sleep(200);
            }
            lift.setTargetPosition(2200);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(700);
            sleep(5000);

        } else if (propX > 600) { // right spike mark
            goTo(0, 0, 90, autoPower, true);

            sleep(500);
            intake.setPower(-1);
            sleep(1000);
            intake.setPower(0);

        } else { // middle spike mark
            goTo(0, 200, 0, autoPower, true);

            sleep(500);
            intake.setPower(-1);
            sleep(1000);
            intake.setPower(0);

            lift.setTargetPosition(1700);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(700);
            goTo(-1000, 800, -110, autoPower, true);
            while (lift.isBusy()) telemetry.addLine("Waiting for lift to reach setpoint...");
            lift.setTargetPosition(1800);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(300);
            goTo(-500, 0, 0, autoPower, true);
            for (int i = 0; i < 1; i++) {
                servo.setPosition(0.20);
                sleep(200);
            }
            lift.setTargetPosition(2200);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(700);
            sleep(5000);

        }

    }
    public void goTo(double forward, double strafe, double yaw, double powercoef, boolean waitToFinish) {
        /**
         * Moves robot-centrically.  All is in ticks except yaw, which is approximately degrees such that 90 turns the robot very approximately 90 degrees clockwise.
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
