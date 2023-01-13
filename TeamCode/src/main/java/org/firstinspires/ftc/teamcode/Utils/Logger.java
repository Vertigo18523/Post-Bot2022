package org.firstinspires.ftc.teamcode.Utils;

import java.util.Formatter;
import java.util.LinkedHashMap;
import java.util.Map;

public class Logger {
    public int level = 1;
    private final Map<String, Integer> dataMap = new LinkedHashMap<>();

    public void log(int level, String data, Object... args){
        dataMap.put(new Formatter().format(data, args).toString(), level);
    }

    public void clear(){
        dataMap.clear();
    }

    public String getData(){
        StringBuilder data = new StringBuilder();
        for (Map.Entry<String, Integer> e : dataMap.entrySet()){
            if(e.getValue() <= level){
                data.append(e.getKey()).append("\n");
            }
        }

        return data.toString();
    }

}