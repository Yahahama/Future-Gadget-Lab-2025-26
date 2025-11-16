package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "FAR_RED_AUTONOMOUS", group = "Autonomous")
public class FarRedAutonomous extends Autonomous {
    @Override
    public void runOpMode() {
        Positions.START startPos = Positions.START.RED_FAR;
        super.runOpMode();
    }
}
