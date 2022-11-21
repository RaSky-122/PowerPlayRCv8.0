package org.firstinspires.ftc.teamcode.rasky.utilities.wrappers;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedList;
import java.util.Queue;

public class WrapperDistanceSensor {

    HardwareMap hardwareMap;
    DistanceSensor distanceSensor;
    String name;

    public WrapperDistanceSensor(HardwareMap hardwareMap, String name) {
        this.hardwareMap = hardwareMap;
        this.name = name;
        distanceSensor = hardwareMap.get(DistanceSensor.class, name);
    }

    double sum = 0;
    int poolingRate = 2500;
    int size = 0;
    double average = -1;
    Queue<Double> distances = new LinkedList<Double>();

    public void update() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        sum += distance;
        distances.offer(distance);
        size++;
        if (size > poolingRate) {
            sum -= distances.remove();
            size--;
            average = sum / poolingRate;
        }
    }

    public double getAverage() {
        return average;
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }

    public void setPoolingRate(int poolingRate) {
        this.poolingRate = poolingRate;
    }

}
