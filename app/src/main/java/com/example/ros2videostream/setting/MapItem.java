package com.example.ros2videostream.setting;

public class MapItem {
    private int position;
    private String settingcontent;
    public MapItem(int position,String settingcontent) {
        this.position=position;
        this.settingcontent=settingcontent;
    }

    public int getPosition() {
        return position;
    }

    public void setPosition(int position) {
        this.position = position;
    }

    public String getSettingcontent() {
        return settingcontent;
    }

    public void setSettingcontent(String settingcontent) {
        this.settingcontent = settingcontent;
    }
}
