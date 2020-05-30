package com.example.ros2camerastream.viewmodel;

import android.app.Application;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.lifecycle.AndroidViewModel;
import androidx.lifecycle.MutableLiveData;

import com.example.ros2camerastream.setting.MapItem;
import com.example.ros2camerastream.setting.Setting;
import com.example.ros2camerastream.setting.SettingArrayList;

import java.util.ArrayList;
import java.util.List;

public class SettingViewModel extends AndroidViewModel {
    private static String logtag = SettingViewModel.class.getName();
    private MutableLiveData<ArrayList<Setting>> settinglist;
    /*private MutableLiveData<MapItem> groupA;
    private MutableLiveData<MapItem> groupB;
    private MutableLiveData<MapItem> groupC;*/
    private MutableLiveData<List<MapItem>> settinggroup;
    public SettingViewModel(@NonNull Application application) {
        super(application);
        SettingArrayList settingArrayList=new SettingArrayList();
        if (settinglist == null) {
            Log.d(logtag,"new setting list");
            settinglist = new MutableLiveData<ArrayList<Setting>>();
            settinglist.setValue(settingArrayList.getSettinglist());
        }
        /*if (groupA == null) {
            groupA = new MutableLiveData<MapItem>();
            groupA.setValue(new MapItem(1,"VGA"));
        }
        if (groupB == null) {
            groupB = new MutableLiveData<MapItem>();
            groupB.setValue(new MapItem(6,"JPG"));
        }
        if (groupC == null) {
            groupC = new MutableLiveData<MapItem>();
            groupC.setValue(new MapItem(8,"Default"));
        }*/
        if (settinggroup == null) {
            settinggroup = new MutableLiveData<List<MapItem>>();
            List<MapItem> group=new ArrayList<>();
            group.add(new MapItem(1,"VGA"));
            group.add(new MapItem(6,"JPG"));
            group.add(new MapItem(8,"Default"));
            settinggroup.setValue(group);
        }
    }

    public MutableLiveData<ArrayList<Setting>> getSettinglist() {
        return settinglist;
    }

    /*public MutableLiveData<MapItem> getGroupA() {
        return groupA;
    }

    public MutableLiveData<MapItem> getGroupB() {
        return groupB;
    }

    public MutableLiveData<MapItem> getGroupC() {
        return groupC;
    }*/

    public MutableLiveData<List<MapItem>> getSettinggroup() {
        return settinggroup;
    }
}
