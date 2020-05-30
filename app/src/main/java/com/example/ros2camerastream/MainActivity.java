package com.example.ros2camerastream;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.FragmentManager;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.Toast;

import com.example.ros2camerastream.ui.CameraxFragment;
import com.example.ros2camerastream.ui.MainFragment;
import com.example.ros2camerastream.ui.SettingFragment;
import com.example.ros2camerastream.ui.TextFragment;

import org.ros2.android.activity.ROSActivity;
import org.ros2.rcljava.RCLJava;

import java.io.File;

public class MainActivity extends ROSActivity {
    private static String logtag = MainActivity.class.getName();
    private FragmentManager manager=getSupportFragmentManager();
    private MainFragment mainFragment;
    private TextFragment textFragment;
    private CameraxFragment cameraxFragment;
    private FrameLayout container;
    private int REQUEST_CODE_PERMISSIONS = 10;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        container=findViewById(R.id.fragment_container);
        //fragment
        if(savedInstanceState!=null){
            mainFragment= (MainFragment) manager.getFragment(savedInstanceState,"mainFragment");
            manager.beginTransaction().show(mainFragment).commit();
            Log.d(logtag,"reload mainFragment");
        }
        else{
            mainFragment=new MainFragment();
            manager.beginTransaction().add(R.id.fragment_container, mainFragment,"MFragment").commit();
        }
        //RCLJava
        RCLJava.rclJavaInit();
        setupStorageDir();
    }

    public void setupStorageDir() {
        ///storage/emulated/0/Android/media/com.example.cameraxapp
        File file = new File(this.getExternalMediaDirs()[0], "Picture");
        if(!file.mkdirs()) {
            Log.d(logtag, "directory not created");
        }
    }

    @Override
    protected void onSaveInstanceState(@NonNull Bundle outState) {
        super.onSaveInstanceState(outState);
        manager.putFragment(outState,"mainFragment",mainFragment);
    }

    @Override
    protected void onResume() {
        super.onResume();
        container.postDelayed(new Runnable() {
            @Override
            public void run() {
                container.setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN|View.SYSTEM_UI_FLAG_HIDE_NAVIGATION|View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
            }
        },500L);
    }

    @Override
    public void onBackPressed() {
        super.onBackPressed();
    }

    public void show_fragment_text(){
        Log.d(logtag,"now show fragment_text");
        textFragment =new TextFragment();
        manager.beginTransaction().addToBackStack("M_to_T").replace(R.id.fragment_container,textFragment,"TFragment").commit();
    }

    public void show_fragment_camerax(){
        //相機權限
        if(checkpermission()){
            //fragment
            setup_camerax_fragment();
        }
        else{
            ActivityCompat.requestPermissions(this,new String[]{Manifest.permission.CAMERA},REQUEST_CODE_PERMISSIONS);
        }

    }

    public void show_fragment_setting(){
        Log.d(logtag,"now show fragment_setting");
        SettingFragment settingFragment =new SettingFragment();
        settingFragment.setTargetFragment(mainFragment, 20);
        manager.beginTransaction().addToBackStack("M_to_S").replace(R.id.fragment_container,settingFragment,"SFragment").commit();
    }

    private boolean checkpermission(){
        return ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED;
    }

    //允許或拒絕後的動作
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if(requestCode==REQUEST_CODE_PERMISSIONS){
            if(checkpermission()){
                //fragment
                setup_camerax_fragment();
            }
            else{
                Toast.makeText(this, "Permissions denied!!", Toast.LENGTH_LONG).show();
            }
        }
    }

    private void setup_camerax_fragment() {
        Log.d(logtag, "now show fragment_camerax");
        cameraxFragment = new CameraxFragment();
        manager.beginTransaction().addToBackStack("M_to_C").replace(R.id.fragment_container, cameraxFragment, "CFragment").commit();
    }
}
