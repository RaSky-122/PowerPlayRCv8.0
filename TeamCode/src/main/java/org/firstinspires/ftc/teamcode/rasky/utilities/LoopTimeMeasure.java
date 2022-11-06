package org.firstinspires.ftc.teamcode.rasky.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Deque;
import java.util.LinkedList;

/**
 * This class measures the average time in ms between each loop.
 *
 * @author Lucian
 * @version 1.0
 */
public class LoopTimeMeasure {

    Telemetry telemetry;

    public LoopTimeMeasure(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    boolean started = false;
    double lastTime = 0;
    double timeSum = 0;
    double poolingRate = 50;

    ElapsedTime timer = new ElapsedTime();
    Deque<Double> times = new LinkedList<Double>();

    /**
     * Call this method every loop to measure the time
     */
    public void measure() {
        //Sets the timer correctly from the first measure
        if (!started) {
            timer.reset();
            started = true;
        }

        //Calculates time difference between loops
        double timeDiff = timer.milliseconds() - lastTime;
        lastTime = timer.milliseconds();

        //Shows average loop time based on the average of all the times in the pooling rate
        timeSum += timeDiff;
        if (times.size() > poolingRate) {
            timeSum -= times.pop();
            telemetry.addData("Avg Time: ", timeSum / poolingRate);
        }
    }

}
