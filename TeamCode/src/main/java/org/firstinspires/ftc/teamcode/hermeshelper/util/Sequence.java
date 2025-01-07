package org.firstinspires.ftc.teamcode.hermeshelper.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Sequence {
    private static class Command {
        Object target;
        double position;
        int delay; // Delay in milliseconds
        
        public Command(Object target, double position, int delay) {
            this.target = target;
            this.position = position;
            this.delay = delay;
        }
        
        public void execute() {
            if (target instanceof ServoV2) {
                // Set position for Servo
                ((ServoV2) target).setPosition(position);
            } else if (target instanceof DcMotorV2) {
                // Run motor to position using manual PIDF
                ((DcMotorV2) target).runToPosition((int) position);
            }
        }
    }
    
    private final Map<String, List<Command>> sequences = new HashMap<>();
    private List<Command> currentSequence;
    private List<Command> activeSequence = null;
    private int activeIndex = 0;
    private final ElapsedTime commandTimer = new ElapsedTime();
    
    public Sequence create(String name) {
        currentSequence = new ArrayList<>();
        sequences.put(name, currentSequence);
        return this;
    }
    
    public Sequence add(Object target, double position, int delay) {
        if (currentSequence != null) {
            currentSequence.add(new Command(target, position, delay));
        }
        return this;
    }
    
    public void build() {
        currentSequence = null;
    }
    
    public void run(String name) {
        if (activeSequence == null) {
            activeSequence = sequences.get(name);
            if (activeSequence != null) {
                activeIndex = 0;
                commandTimer.reset();
            }
        }
    }
    
    public void update() {
        if (activeSequence != null && activeIndex < activeSequence.size()) {
            Command currentCommand = activeSequence.get(activeIndex);
            
            // Only execute the next command if the timer has passed the current command's delay
            if (commandTimer.milliseconds() >= currentCommand.delay) {
                currentCommand.execute();
                activeIndex++;
                commandTimer.reset(); // Reset the timer after executing a command
            }
        }
        
        // Reset if the sequence is complete
        if (activeSequence != null && activeIndex >= activeSequence.size()) {
            activeSequence = null;
            activeIndex = 0;
        }
    }
    
    public boolean isRunning() {
        return activeSequence != null;
    }
}
