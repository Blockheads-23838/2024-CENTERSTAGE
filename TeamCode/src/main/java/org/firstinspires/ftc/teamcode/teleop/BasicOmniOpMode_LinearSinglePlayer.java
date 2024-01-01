package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Constants;

@TeleOp(name="One Person Omni Linear OpMode", group="Linear OpMode")
public class BasicOmniOpMode_LinearSinglePlayer extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private DcMotorEx lift = null;
    private DcMotorEx trident = null;
    private Servo leftTridentServo = null;
    private Servo rightTridentServo = null;
    private IMU imu = null;
    // private Servo servo = null;
    private Servo crossbow = null;
    double powercoef = 0.5;
    private boolean fieldCentric = false;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");

        imu = hardwareMap.get(IMU.class, "imu");

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        intake = hardwareMap.get(DcMotor.class, "intake");
        trident = hardwareMap.get(DcMotorEx.class, "trident");

        leftTridentServo = hardwareMap.get(Servo.class, "trident_left");
        rightTridentServo = hardwareMap.get(Servo.class, "trident_right");

        crossbow = hardwareMap.get(Servo.class, "crossbow");

        leftFrontDrive.setDirection(Constants.motorDirections.get("left_front"));
        leftBackDrive.setDirection(Constants.motorDirections.get("left_back"));
        rightFrontDrive.setDirection(Constants.motorDirections.get("right_front"));
        rightBackDrive.setDirection(Constants.motorDirections.get("right_back"));
        lift.setDirection(Constants.motorDirections.get("lift"));

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imu.resetYaw();

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Lift joystick position: ", gamepad2.right_stick_y);
        }

        runtime.reset();
        // since this opmode is for testing
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTridentServo.setPosition(Constants.leftTridentClosedPosition);
        rightTridentServo.setPosition(Constants.rightTridentClosedPosition);
        trident.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trident.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // arm servo debug code
            // arm.setPower(gamepad2.right_stick_y * 1/4);
            // servo.setPosition(position);
            // intake.setPower(-gamepad2.right_trigger);

            // crossbow
            double crossbowSetpoint;
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                telemetry.addLine("crossbowing!!");
                crossbowSetpoint = 0.0;
            } else {
                crossbowSetpoint = 0.2;
            }
            telemetry.addData("crossbow setpoint", crossbowSetpoint);
            if (crossbowSetpoint != crossbow.getPosition()) crossbow.setPosition(crossbowSetpoint);

            if (gamepad1.b) imu.resetYaw();

            handleLift();

            HandleDrivetrain();
            telemetry.update();
        }
    }

    public void handleLift() {
        // Lift stuff
        if (gamepad2.a) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        int liftPos = lift.getCurrentPosition();
        telemetry.addData("lift position: ", liftPos);
        double liftPower;
        boolean coasting = false;
        if (gamepad2.left_bumper) liftPower = gamepad2.right_stick_y * 1500; // manual override; also former ticks/sec value
        else if (liftPos > Constants.TopLiftPosition && -gamepad2.right_stick_y < 0) liftPower = 0;
        else if (gamepad2.right_trigger > 0.5) {
            int error = Constants.groundLiftPosition - lift.getCurrentPosition();
            if (error < 10) { coasting = true; liftPower = 0; }
            else liftPower = (Constants.groundLiftPosition - lift.getCurrentPosition()); // P controller to get the lift down
        }
        else if (liftPos < Constants.groundLiftPosition) liftPower = 200;
        else liftPower = gamepad2.right_stick_y * 2500;

        if (coasting) lift.setPower(0);
        else lift.setVelocity(liftPower + 1);

        // Intake stuff
        if (gamepad2.right_trigger > 0.5) {
            intake.setPower(-1);
        } else {
            intake.setPower(gamepad2.left_trigger);
        }

        // Trident motor stuff
        if (lift.getCurrentPosition() > Constants.tridentDeployLiftPosition && -gamepad2.right_stick_y <= 0.5 ||
                lift.getCurrentPosition() > Constants.tridentStowPosition){
            // if we're above the deploy position and going up OR we're above the stow position, deploy the trident
            trident.setVelocity((Constants.tridentDeployedPosition - trident.getCurrentPosition()) * Constants.tridentk_P);
        } else {
            trident.setVelocity(-trident.getCurrentPosition() * Constants.tridentk_P);
        }

        // Trident servo stuff
        double leftTridentSetpoint = Constants.leftTridentClosedPosition;
        double rightTridentSetpoint = Constants.rightTridentClosedPosition;;

        /*
        if (gamepad2.left_stick_x < -0.2) {
            telemetry.addLine("Opening left trident");
            leftTridentSetpoint = Constants.leftTridentOpenPosition;
        } else if (gamepad2.left_stick_x > 0.2) {
            telemetry.addLine("Opening right trident");
            rightTridentSetpoint = Constants.rightTridentOpenPosition;
        } else */ if (gamepad2.y || gamepad2.right_bumper) {
            leftTridentSetpoint = Constants.leftTridentOpenPosition;
            rightTridentSetpoint = Constants.rightTridentOpenPosition;
        }
        telemetry.addData("left trident setpoint: ", leftTridentSetpoint);
        telemetry.addData("right trident setpoint: ", rightTridentSetpoint);

        if (leftTridentServo.getPosition() != leftTridentSetpoint) leftTridentServo.setPosition(leftTridentSetpoint);
        if (rightTridentServo.getPosition() != rightTridentSetpoint) rightTridentServo.setPosition(rightTridentSetpoint);

        // trident.setPower(gamepad2.left_stick_y);
        telemetry.addData("trident position: ", trident.getCurrentPosition());

    }

    public void HandleDrivetrain() {
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

        if (gamepad1.y) fieldCentric = !fieldCentric;
        if (gamepad1.b) imu.resetYaw();
        telemetry.addData("FIELD_CENTRIC: ", fieldCentric);

        if(gamepad2.x) powercoef = 1;
        else if (gamepad2.left_trigger > 0.3) powercoef = 0.2;
        else powercoef = 0.4;

        double forward = -gamepad2.left_stick_y; /* Invert stick Y axis */
        double strafe = 0;
        double yaw = gamepad2.left_stick_x;

        if (fieldCentric) {
            double gyro_radians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("gyro angle: ", gyro_radians);
            strafe = -strafe;
            double temp = forward * Math.cos(gyro_radians) +
                    strafe * Math.sin(gyro_radians);
            strafe = -forward * Math.sin(gyro_radians) +
                    strafe * Math.cos(gyro_radians);
            strafe *= -1;
            forward = temp;

            /* At this point, Joystick X/Y (strafe/forwrd) vectors have been */
            /* rotated by the gyro angle, and can be sent to drive system */
        }


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = powercoef * (forward + strafe + yaw);
        double rightFrontPower = powercoef * (forward - strafe - yaw);
        double leftBackPower = powercoef * (forward - strafe + yaw);
        double rightBackPower = powercoef * (forward + strafe - yaw);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        telemetry.addData("left front pos: ", leftFrontDrive.getCurrentPosition());
        telemetry.addData("left back pos: ", leftBackDrive.getCurrentPosition());
        telemetry.addData("right front pos: ", rightFrontDrive.getCurrentPosition());
        telemetry.addData("right back pos: ", rightBackDrive.getCurrentPosition());

    }
}
