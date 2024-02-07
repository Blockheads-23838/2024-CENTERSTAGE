/* just telling android studio that this file is inside this directory */
package org.firstinspires.ftc.teamcode.common;

/* 
any packages we need to import for our code - these can't actually 
be found in the codebase and are instead fetched by android studio 
when you build your program 
*/

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/* every java file needs a class that matches its name! */
public class RobotContainer {
    /*
    This class is very important. It basically contains all the objects and methods that we'll use
    across different programs. It should also contain methods such as DriveForwards to reduce
     boilerplate code, but we didn't have time to do that in 2023.
     */

    /* 
    declare the motors and servo and set them to null
    note that not all of these motors may be applicable to
    your season!!
    */
    private LinearOpMode linearOpmode;
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotorEx leftFrontDrive = null; // left front motor/wheel
    public DcMotorEx rightFrontDrive = null; // right front motor/wheel
    public DcMotorEx leftBackDrive = null; // left back motor/wheel
    public DcMotorEx rightBackDrive = null; // right back motor/wheel
    public ArrayList <DcMotorEx> driveMotors;
    public DcMotorEx lift = null;
    public DcMotor intake = null;
    public Servo servo = null;
    public  Servo autoHook = null;
    public Servo crossbow = null;
    private DcMotorEx climberDownstairs = null;
    private DcMotorEx climberUpstairs = null;
    public IMU imu = null;

    /* declare our gyro (imu) and camera */

    /**
     * The initialization method used in every driveMode and
     * opMode to define our hardware. Put this inside runOpMode
     * and pass to it the hardwareMap abstract.
     *
     * @param linOpmode Pass "this" when initting the robot.
     */
    public void init(LinearOpMode linOpmode) {
        linearOpmode = linOpmode;
        HardwareMap hardwareMap = linOpmode.hardwareMap;

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

        // Scoring activation
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        intake = hardwareMap.get(DcMotor.class, "intake");

        lift.setDirection(Constants.motorDirections.get("lift"));
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setVelocity(0.001);

        servo = hardwareMap.get(Servo.class, "servo");

        // Auto + endgame activation
        autoHook = hardwareMap.get(Servo.class, "auto_hook");
        autoHook.setPosition(Constants.autoHookStowPosition);

        crossbow = hardwareMap.get(Servo.class, "crossbow");

        climberDownstairs = hardwareMap.get(DcMotorEx.class, "climber_downstairs");
        climberUpstairs = hardwareMap.get(DcMotorEx.class, "climber_upstairs");
    }

    public void goToVector(double directionDegrees, double magnitude, double powercoef, boolean waitToFinish) {
        double directionRadians = 3.14159265358979323 * 2 / 360;
        goTo(magnitude * Math.cos(directionRadians), Math.sin(-directionRadians), 0, powercoef, waitToFinish);
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
            linearOpmode.telemetry.addData("lf power: ", leftFrontPower);
            linearOpmode.telemetry.addData("lf tgt position: ", leftFrontDrive.getTargetPosition());
            linearOpmode.telemetry.addData("lf position: ", leftFrontDrive.getCurrentPosition());
            linearOpmode.telemetry.update();
        }
    }

}

