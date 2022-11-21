package org.firstinspires.ftc.teamcode.rasky.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Deque;
import java.util.LinkedList;
import java.util.Queue;

/**
 * This class measures the average time in ms between each loop.
 *
 * @author Lucian
 * @version 1.2
 */
public class LoopTimeMeasure {

    Telemetry telemetry;

    public LoopTimeMeasure(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    boolean started = false;
    double lastTime = 0;
    double timeSum = 0;
    int poolingRate = 50;
    int size = 0;

    ElapsedTime timer = new ElapsedTime();
    Queue<Double> times = new LinkedList<Double>();

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
        times.offer(timeDiff);
        size++;
        if (size > poolingRate) {
            size--;
            timeSum -= times.remove();
            telemetry.addData("Avg Time: ", timeSum / poolingRate);
        }
    }

    /**
     * Change the pooling rate of the loop measurer.
     * <p>
     * Default pooling rate is 50 loops.
     * @param poolingRate New pooling rate
     */
    public void setPoolingRate(int poolingRate) {
        this.poolingRate = poolingRate;
    }

}
