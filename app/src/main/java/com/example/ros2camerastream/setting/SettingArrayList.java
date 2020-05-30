package com.example.ros2camerastream.setting;

import java.util.ArrayList;

public class SettingArrayList {
    private ArrayList<Setting> settinglist;

    public SettingArrayList() {
        settinglist=new ArrayList<>();
        settinglist.add(new Setting("圖片解析度",false,"圖片解析度","A"));
        settinglist.add(new Setting("VGA 640x480 (Default)",true,"VGA","B"));
        settinglist.add(new Setting("HD 1280x720",false,"HD","B"));
        settinglist.add(new Setting("FullHD 1920x1080",false,"FullHD","B"));
        settinglist.add(new Setting("圖片格式",false,"圖片格式","A"));
        settinglist.add(new Setting("PNG",false,"PNG","B"));
        settinglist.add(new Setting("JPG (Default)",true,"JPG","B"));
        settinglist.add(new Setting("Ros2 QOS 模式",false,"Ros2 QOS 模式","A"));
        settinglist.add(new Setting("Default (Reliable) (Default)",true,"Default","B"));
        settinglist.add(new Setting("Services (Reliable)",false,"Services","B"));
        settinglist.add(new Setting("Parameters (Reliable)",false,"Parameters","B"));
        settinglist.add(new Setting("Sensor data (Best effort)",false,"Sensor","B"));
    }

    public ArrayList<Setting> getSettinglist() {
        return settinglist;
    }
}
