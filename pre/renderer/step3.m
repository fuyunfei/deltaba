%% last step ,generate Myfile.txt pos2d pos3d  vx vy  
%  vx vy represents  (pos_queryI)+(vx vy)= pos_model

load('PreCeres.mat'); % 注意其中unproject.obj 已经修改 没脖子 没耳朵
 

%% 组织架构 :  vertex 2d positon,vertex 3d position,vx,vy  
[vertex,face]=read_obj_file('unproject.obj');
unproj_rt=[[est_R est_T'] * [vertex ones(size(vertex,1),1)]']';
unproj_art=est_A*unproj_rt';

%unproj_art : 第一行为照片横坐标 第二行为照片纵坐标
for i=1:size(unproj_art,2)
unproj_art(1,i)=unproj_art(1,i)/unproj_art(3,i);
unproj_art(2,i)=unproj_art(2,i)/unproj_art(3,i);
end

modi= zeros(size(unproj_art,2),2);
unproj_art=int16(unproj_art);
for i=1:size(unproj_art,2)
    modi(i,1)= vx(unproj_art(2,1),unproj_art(1,i));
    modi(i,2)= vy(unproj_art(2,1),unproj_art(1,i));
end 
pos2d=double(unproj_art(1:2,:)');
file = [pos2d vertex modi];

formatSpec = '%d %d %f %f %f %d %d \n';
fid=fopen('MyFile.txt','w');
fprintf(fid, formatSpec, file');
fclose(fid);