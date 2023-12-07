package org.firstinspires.ftc.teamcode.Season;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Config
public class Z_Global_Variables {

//---------------------------------------------------------------------------

    //Auto Variables
    public static int AutoLiftSetPt = 100;
//    public static int LiftSetPtIntake = -5;
//    public static double WristSetPtIn = 0.38;
//    public static double WristSetPtScore = 0.44;
//    public static double ClawSetPtSingleSmall = 0.94;
    public static double AutoClawSetPtOpen = 0.7;

//---------------------------------------------------------------------------

    //Motor Set Points
    public static int LiftSetPtIntake = -10;
    public static int LiftSetPtLvl1 = 350;
    public static int LiftSetPtLvl2 = 700;
    public static int LiftSetPtLvl3 = 1050;
    public static int LiftSetPtLvl4 = 1400;
//    public static int LiftSetPtLvl4 = 2200;

    public static int ClimbSetPtUp = -2150;
    public static int ClimbSetPtDown = 0;

//---------------------------------------------------------------------------

    //Servo Set Points
    public static double WristSetPtIn = 0.38;
    public static double WristSetPtOut = 0.64;
    public static double WristSetPtScore = 0.44;

    public static double ClawSetPtClosed = 0.88;
    public static double ClawSetPtOpen = 0.75;
    public static double ClawSetPtSingleSmall = 0.94;

    public static double DroneSetPtClosed = 0.5;
    public static double DroneSetPtOpen = 0.59;

    public static double HookSetPtClosed = 0.5;
    public static double HookSetPtOpen = 0.35;

//---------------------------------------------------------------------------

    //Auto pick up sleeps

    public static int WristSleepDown = 400;
    public static int WristSleepUp = 0;
    public static int WristSleepDownSmall = 400;
    public static int WristSleepUpSmall = 200;
    public static int WristSleepBack = 100;
}
