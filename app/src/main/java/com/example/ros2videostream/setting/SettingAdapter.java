package com.example.ros2videostream.setting;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckedTextView;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import com.example.ros2videostream.R;
import com.example.ros2videostream.viewmodel.SettingViewModel;

import java.util.ArrayList;
import java.util.HashMap;

public class SettingAdapter extends RecyclerView.Adapter<RecyclerView.ViewHolder>{
    private static final int TYPE_TITLE=1;
    private static final int TYPE_CONTENT=2;
    private Context context;
    private ArrayList<Setting> settinglist;
    private SettingViewModel settingViewModel;
    private HashMap<String, MapItem> checkmap;

    public SettingAdapter(Context context, ArrayList<Setting> settinglist,SettingViewModel settingViewModel) {
        this.context=context;
        this.settinglist=settinglist;
        this.settingViewModel=settingViewModel;
        checkmap=new HashMap<>();
        checkmap.put("groupA",settingViewModel.getGroupA().getValue());
        checkmap.put("groupB",settingViewModel.getGroupB().getValue());
        checkmap.put("groupC",settingViewModel.getGroupC().getValue());
    }

    //建立ViewHolder
    class TitleViewHolder extends RecyclerView.ViewHolder{
        private TextView title;
        public TitleViewHolder(@NonNull View itemView) {
            super(itemView);
            title = itemView.findViewById(R.id.title);
        }
    }
    class ContentViewHolder extends RecyclerView.ViewHolder{
        private CheckedTextView settingitem;
        public ContentViewHolder(@NonNull View itemView) {
            super(itemView);
            settingitem=itemView.findViewById(R.id.setting_item);
        }
    }

    @NonNull
    @Override
    public RecyclerView.ViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        View view;
        if(viewType==TYPE_TITLE){
            view = LayoutInflater.from(context).inflate(R.layout.setting_recycleview_item_a, parent, false);
            return new TitleViewHolder(view);
        }
        else{
            view = LayoutInflater.from(context).inflate(R.layout.setting_recycleview_item_b, parent, false);
            return new ContentViewHolder(view);
        }
    }

    @Override
    public void onBindViewHolder(@NonNull RecyclerView.ViewHolder holder, int position) {
        Setting setting=settinglist.get(position);
        if(getItemViewType(position)==TYPE_TITLE){
            ((TitleViewHolder)holder).title.setText(setting.getSettingitem());
        }
        else {
            ((ContentViewHolder)holder).settingitem.setText(setting.getSettingitem());
            //如果item被回收先重新設定
            if(setting.isIscheck()){
                ((ContentViewHolder)holder).settingitem.setChecked(true);
            }
            else {
                ((ContentViewHolder)holder).settingitem.setChecked(false);
            }
            ((ContentViewHolder)holder).itemView.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    if(position>0 && position<4){
                        //groupA
                        //先取消上一個位置的checkBox
                        MapItem item=checkmap.get("groupA");
                        if(item!=null){
                            Setting cancel=settinglist.get(item.getPosition());
                            cancel.setIscheck(false);
                            notifyItemChanged(item.getPosition());
                        }
                        //更新目前位置
                        ((ContentViewHolder)holder).settingitem.setChecked(true);
                        setting.setIscheck(true);
                        item=new MapItem(position,setting.getSettingcontent());
                        checkmap.put("groupA",item);
                        settingViewModel.getGroupA().setValue(item);
                    }
                    else if(position>4 && position<7){
                        //groupB
                        MapItem item=checkmap.get("groupB");
                        if(item!=null){
                            Setting cancel=settinglist.get(item.getPosition());
                            cancel.setIscheck(false);
                            notifyItemChanged(item.getPosition());
                        }
                        //更新目前位置
                        ((ContentViewHolder)holder).settingitem.setChecked(true);
                        setting.setIscheck(true);
                        item=new MapItem(position,setting.getSettingcontent());
                        checkmap.put("groupB",item);
                        settingViewModel.getGroupB().setValue(item);
                    }
                    else {
                        //groupC
                        MapItem item=checkmap.get("groupC");
                        if(item!=null){
                            Setting cancel=settinglist.get(item.getPosition());
                            cancel.setIscheck(false);
                            notifyItemChanged(item.getPosition());
                        }
                        ((ContentViewHolder)holder).settingitem.setChecked(true);
                        setting.setIscheck(true);
                        item=new MapItem(position,setting.getSettingcontent());
                        checkmap.put("groupC",item);
                        settingViewModel.getGroupC().setValue(item);
                    }
                }
            });
        }
    }

    @Override
    public int getItemCount() {
        if(settinglist!=null){
            return settinglist.size();
        }
        else return 0;
    }

    @Override
    public int getItemViewType(int position) {
        //return super.getItemViewType(position);
        if(settinglist.get(position).getViewtype().equals("A")){
            return TYPE_TITLE;
        }
        else {
            return TYPE_CONTENT;
        }
    }
}
