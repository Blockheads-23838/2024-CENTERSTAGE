package org.firstinspires.ftc.teamcode.common;

import java.util.HashMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@Config
public class Constants {
    // this is a very useful class. Put constants here that you'll use across multiple programs
    // (e.g. gear ratios, target positions) so that you don't have to change the values in all the programs

    // This saves all the directions in one place, so we don't have to change directions in all our code if we ever have to change directions.
    // The hashmap lets us keep track of which values go with which wheels.
    public static final HashMap<String, Direction> motorDirections = new HashMap<String, Direction>() {{
        put("left_front", Direction.REVERSE);
        put("right_front", Direction.FORWARD);
        put("left_back", Direction.REVERSE);
        put("right_back", Direction.FORWARD);
        put("lift", Direction.REVERSE);
    }};
    public static int TopLiftPosition = 3060;
    public static int tridentDeployLiftPosition = 800;
    public static int tridentStowPosition = 1200;
    public static int groundLiftPosition = 0;

    public static double tridentk_P = 2;
    public static int tridentDeployedPosition = 1200;
    public static double leftTridentOpenPosition = 0.55;
    public static double rightTridentOpenPosition = 0.55;
    public static double leftTridentClosedPosition = 0.4;
    public static double rightTridentClosedPosition = 0.7;
}
