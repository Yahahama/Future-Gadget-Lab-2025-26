package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "CLOSE_RED_AUTONOMOUS", group = "Autonomous")
public class CloseRedAutonomous extends Autonomous {
    @Override
    public void runOpMode() {
        Positions.START startPos = Positions.START.RED_CLOSE;
        super.runOpMode();
    }
}
