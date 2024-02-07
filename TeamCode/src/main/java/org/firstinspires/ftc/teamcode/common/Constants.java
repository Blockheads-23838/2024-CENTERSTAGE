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
    public static final int TopLiftPosition = 3060;
    public static final int ClearIntakeLiftPosition = 800;
    public static final int IntakingLiftPosition = 120;
    public static final int groundLiftPosition = 0;
    public static double crossbowRestPosition = 1;
    public static double crossbowFirePosition = 0.8;
    public static double autoHookStowPosition = 0.8; // 0.8 is good
    public static double autoHookPurpleDropPosition = 0.58; // 0.58 is good
    public static double autoHookYellowDropPosition = 0; //0 is ok

    public static double climberDownstairsGoToZeroPosition = 120;


    public static double IntakingServoPosition = 0.05; // 0.05 works great in solo (only leo) testing
}
