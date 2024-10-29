package org.firstinspires.ftc.teamcode.hermeshelper.util.hardware;

import java.util.ArrayList;

//public class Part {
//    private Map<String, Object> parts = new HashMap<>();
//
//    public Part add(String name, Object object) {
//        parts.put(name, object);
//        return this;
//    }
//
//    public Object get(String name) {
//        return parts.get(name);
//    }
//}

public class Part {
    private ArrayList<Object> parts = new ArrayList<>();

    public Part add(Object object) {
        parts.add(object);
        return this;
    }

    public Object get(Object object) {
        return parts.get(parts.indexOf(object));
    }
}