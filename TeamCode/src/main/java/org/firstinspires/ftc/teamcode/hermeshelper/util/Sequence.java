package org.firstinspires.ftc.teamcode.hermeshelper.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Sequence {
    // Inner class to represent a command with its target, position, and delay
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
            // Set the target's position based on type
            if (target instanceof ServoV2) {
                ((ServoV2) target).setPosition(position);
            } else if (target instanceof DcMotorV2) {
                ((DcMotorV2) target).runToPosition((int) position);
            }
        }
    }

    // Store sequences by name
    private final Map<String, List<Command>> sequences = new HashMap<>();
    private List<Command> currentSequence;

    // Method to create a new sequence by name
    public Sequence create(String name) {
        currentSequence = new ArrayList<>();
        sequences.put(name, currentSequence);
        return this;
    }

    // Method to add a command to the current sequence
    public Sequence add(Object target, double position, int delay) {
        if (currentSequence != null) {
            currentSequence.add(new Command(target, position, delay));
        }
        return this;
    }

    // Method to finalize the current sequence
    public void build() {
        currentSequence = null;
    }

    // Method to run a sequence by name
    public void run(String name) {
        List<Command> sequence = sequences.get(name);
        if (sequence != null) {
            ElapsedTime timer = new ElapsedTime();
            for (Command command : sequence) {
                command.execute();

                // Wait for the specified delay without blocking
                timer.reset();
                while (timer.milliseconds() < command.delay) {
                    // You can add telemetry updates here if needed
                }
            }
        } else {
            System.out.println("Sequence not found: " + name);
        }
    }
}
