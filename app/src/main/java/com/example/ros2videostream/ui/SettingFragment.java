package com.example.ros2videostream.ui;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.example.ros2videostream.R;
import com.example.ros2videostream.setting.Setting;
import com.example.ros2videostream.setting.SettingAdapter;
import com.example.ros2videostream.viewmodel.SettingViewModel;

import java.util.ArrayList;

public class SettingFragment extends Fragment {
    private View view;

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        if(view==null){
            view = inflater.inflate(R.layout.fragment_setting, null);
            Log.d("view","load fragment_setting");
        }
        //return super.onCreateView(inflater, container, savedInstanceState);
        return view;
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        Toolbar toolbar =view.findViewById(R.id.toolbar_setting);
        ((AppCompatActivity) getActivity()).setSupportActionBar(toolbar);
        toolbar.setTitle(R.string.toolbar_setting_title);
        toolbar.setNavigationIcon(R.drawable.ic_arrow_back_black_24dp);
        toolbar.setNavigationOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                requireActivity().onBackPressed();
            }
        });
        //關聯ViewModel
        SettingViewModel settingViewModel = new ViewModelProvider(requireActivity()).get(SettingViewModel.class);
        ArrayList<Setting> settinglist = settingViewModel.getSettinglist().getValue();
        //新增recyclerview
        RecyclerView recyclerView = view.findViewById(R.id.recycleview);
        SettingAdapter settingAdapter = new SettingAdapter(requireContext(), settinglist, settingViewModel);
        recyclerView.setLayoutManager(new LinearLayoutManager(requireContext()));
        recyclerView.setAdapter(settingAdapter);
        /*Log.e("SgroupA", settingViewModel.getGroupA().getValue().getSettingcontent());
        Log.e("SgroupB", settingViewModel.getGroupB().getValue().getSettingcontent());
        Log.e("SgroupC", settingViewModel.getGroupC().getValue().getSettingcontent());*/
    }
}
