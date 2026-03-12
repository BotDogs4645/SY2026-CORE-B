package frc.robot;

import java.util.HashMap;
import java.util.List;

public class ShiftConstants {
    
    // TODO : make the hubs active var change between auto winners and losers

    public static enum shifts {
        AUTO,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        END_GAME
    };

    public static final shifts[] SHIFTS_ORDER = {
        shifts.AUTO,
        shifts.TRANSITION,
        shifts.SHIFT_1,
        shifts.SHIFT_2,
        shifts.SHIFT_3,
        shifts.SHIFT_4,
        shifts.END_GAME
    };
    public static enum hubsOpen {
        RED_ALLIANCE,
        BLUE_ALLIANCE,
        BOTH,
        NONE
    };
    public static final HashMap<shifts, Integer> SHIFT_LENGTHS = new HashMap<shifts, Integer>(){{
        put(shifts.AUTO, 20);
        put(shifts.TRANSITION, 10);
        put(shifts.SHIFT_1, 25);
        put(shifts.SHIFT_2, 25);
        put(shifts.SHIFT_3, 25);
        put(shifts.SHIFT_4, 25);
        put(shifts.END_GAME, 25);
    }};

    public static final int SHIFT_WARNING_TIME =  5;

    public static final HashMap<shifts, hubsOpen> SHIFT_HUBS = new HashMap<shifts, hubsOpen>(){{
        put(shifts.AUTO,        hubsOpen.BOTH);
        put(shifts.TRANSITION,  hubsOpen.BOTH);
        put(shifts.SHIFT_1,     hubsOpen.BLUE_ALLIANCE);
        put(shifts.SHIFT_2,     hubsOpen.RED_ALLIANCE);
        put(shifts.SHIFT_3,     hubsOpen.BLUE_ALLIANCE);
        put(shifts.SHIFT_4,     hubsOpen.RED_ALLIANCE);
        put(shifts.END_GAME,    hubsOpen.BOTH);
    }};
    public static final HashMap<shifts, shifts> SHIFT_NEXT = new HashMap<shifts, shifts>(){{
        put(shifts.AUTO,        shifts.TRANSITION);
        put(shifts.TRANSITION,  shifts.SHIFT_1);
        put(shifts.SHIFT_1,     shifts.SHIFT_2);
        put(shifts.SHIFT_2,     shifts.SHIFT_3);
        put(shifts.SHIFT_3,     shifts.SHIFT_4);
        put(shifts.SHIFT_4,     shifts.END_GAME);
        put(shifts.END_GAME,    shifts.END_GAME);
    }};
    public static final HashMap<shifts, String> SHIFT_NAMES = new HashMap<shifts, String>(){{
        put(shifts.AUTO,        "AUTO");
        put(shifts.TRANSITION,  "TRANSITION");
        put(shifts.SHIFT_1,     "SHIFT 1");
        put(shifts.SHIFT_2,     "SHIFT 2");
        put(shifts.SHIFT_3,     "SHIFT 3");
        put(shifts.SHIFT_4,     "SHIFT 4");
        put(shifts.END_GAME,    "END GAME");
    }};
    public static final shifts getShift(int time){
        int timeCounter = 0;
        for (shifts sh : SHIFTS_ORDER){
            int shiftTime = SHIFT_LENGTHS.get(sh);
            if (time >=timeCounter && time <shiftTime+timeCounter){
                return sh;
            }
            timeCounter+=shiftTime;
        };
        return null;
    }
    



}
