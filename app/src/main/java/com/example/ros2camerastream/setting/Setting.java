package com.example.ros2camerastream.setting;

public class Setting {
    private String settingitem;
    private boolean ischeck;
    private String settingcontent;
    private String  viewtype;

    public Setting(String settingitem, boolean ischeck,String settingcontent, String viewtype) {
        this.settingitem = settingitem;
        this.ischeck = ischeck;
        this.settingcontent=settingcontent;
        this.viewtype=viewtype;
    }

    public String getSettingitem() {
        return settingitem;
    }

    public void setSettingitem(String settingitem) {
        this.settingitem = settingitem;
    }

    public boolean isIscheck() {
        return ischeck;
    }

    public String getSettingcontent() {
        return settingcontent;
    }

    public void setSettingcontent(String settingcontent) {
        this.settingcontent = settingcontent;
    }

    public void setIscheck(boolean ischeck) {
        this.ischeck = ischeck;
    }

    public String getViewtype() {
        return viewtype;
    }

    public void setViewtype(String viewtype) {
        this.viewtype = viewtype;
    }
}
