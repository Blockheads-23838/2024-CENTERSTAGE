package org.firstinspires.ftc.teamcode.common;

public class RobotConstantsCenterStage {

    public enum OpMode {
        // Autonomous OpModes
        BLUE_A2(OpModeType.COMPETITION),
        BLUE_A4(OpModeType.COMPETITION),
        RED_F2(OpModeType.COMPETITION),
        RED_F4(OpModeType.COMPETITION),

        TEST(OpModeType.AUTO_TEST), TEST_PRE_MATCH(OpModeType.AUTO_TEST),
        TEST_ELEVATOR(OpModeType.AUTO_TEST),
        AUTO_NO_DRIVE(OpModeType.AUTO_TEST),

        // TeleOp OpModes
        TELEOP_NO_DRIVE(OpModeType.TELEOP_TEST),

        // Pseudo OpModes for running Autonomous actions from within
        // TeleOp. These are not "real" OpMoces in that they don't
        // appear on the Driver Station but they are present in
        // RobotAction.xml.
        TELEOP_TAKE_PICTURE_WEBCAM(OpModeType.PSEUDO_OPMODE),

        // Pseudo OpModes for running EasyOpenCV webcam calibration
        // from TeleOp. These are also not "real" OpMoces in that
        // they don't appear on the Driver Station but they are
        // present in RobotAction.xml.
        TEAM_PROP_CALIBRATION(OpModeType.PSEUDO_OPMODE);

        public enum OpModeType {COMPETITION, AUTO_TEST, TELEOP_TEST, PSEUDO_OPMODE}
        private final OpModeType opModeType;

        OpMode(OpModeType pOpModeType) {
            opModeType = pOpModeType;
        }

        public OpModeType getOpModeType() {
            return opModeType;
        }
    }

    // The CameraId identifies each unique camera and its position on
    // the robot.
    public enum InternalWebcamId {
        FRONT_WEBCAM, REAR_WEBCAM, WEBCAM_NPOS
    }

    public enum ProcessorIdentifier {
        RAW_FRAME, APRIL_TAG, CAMERA_STREAM_PREVIEW, PROCESSOR_NPOS
    }

    public enum TeamPropRecognitionPath {
        COLOR_CHANNEL_CIRCLES, COLOR_CHANNEL_PIXEL_COUNT,
        COLOR_CHANNEL_BRIGHT_SPOT, GRAYSCALE_BRIGHT_SPOT
    }

    // Relative position of a barcode element within the ROI.
    public enum SpikeLocationWindow {
        LEFT, RIGHT, WINDOW_NPOS
    }

    // Constructor parameters are the AprilTag id of the
    // blue alliance backstop locations and the AprilTag
    // id of the red alliance backstop locations.
    public enum TeamPropLocation {
        LEFT_SPIKE(1, 4), CENTER_SPIKE (2, 5), RIGHT_SPIKE(3, 6), SPIKE_NPOS(-1, -1);

        private final int blueBackdropAprilTagId;
        private final int redBackdropAprilTagId;

        TeamPropLocation(int pBlueBackdropAprilTagId, int pRedBackdropAprilTagId) {
            blueBackdropAprilTagId = pBlueBackdropAprilTagId;
            redBackdropAprilTagId = pRedBackdropAprilTagId;
        }

        //## The FTCAuto action DRIVE_TO_APRIL_TAG uses the AprilTag id but could
        // use the alliance and the spike id to call one of the methods here.
        public int getBlueBackdropAprilTagId() {
            return blueBackdropAprilTagId;
        }

        public int getRedBackdropAprilTagId() {
            return redBackdropAprilTagId;
        }
    }

    public enum BackdropPixelRecognitionPath {
        RED_CHANNEL_GRAYSCALE
    }

    public enum BackdropPixelOpenSlot {
        LEFT, RIGHT, ANY_OPEN_SLOT, OPEN_SLOT_NPOS
    }

    public enum AutoEndingPosition {LEFT, RIGHT}

}