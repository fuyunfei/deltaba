%%  PreCeres step 1: generate  unproj 
clear all, close all,
%load face_p99.mat
load face_p99.mat;
model_name='../Rock/TheRock2.obj';

render_width = 540;
render_height = 540;

[refD, refI, refU, outA, outR, outT] = renderer(render_width,render_height ,model_name , 0, 1, 1, 10, 0, 0, 'zxy'); 
% detect facial features on the rendered image, code from www.ics.uci.edu/~xzhu/face/ with modifications to get center of detection window (detect parameters taken from their code)

figure, imshow(refI);
ref_bs = detect(refI, model, 0.6);% 

ref_bs = clipboxes(refI, ref_bs);
ref_bs = nms_face(ref_bs,0.3);
posemap = 90:-15:-90;

%figure,showboxes(refI, ref_bs(1),posemap),title('Highest scoring detection');
hold on;
x1 = ref_bs.xy(:,1);
y1 = ref_bs.xy(:,2);
x2 = ref_bs.xy(:,3);
y2 = ref_bs.xy(:,4);
ref_XY = [(x1+x2)/2,(y1+y2)/2];
x3 = ref_XY(:,1);
y3 = ref_XY(:,2);

scatter(x3,y3,20,[1,0,1],'filled'); hold on; 
%scatter(x2,y2,20,[0,1,1],'filled');


%show refU refD
%figure, imshow(refD);
 refUX=reshape(refU(:,:,1),1,render_width*render_height);
 refUY=reshape(refU(:,:,2),1,render_width*render_height);
 refUZ=reshape(refU(:,:,3),1,render_width*render_height);
%  figure('position', [0, 0, 1000, 1000]);
%  scatter3(refUX,refUY,refUZ,1,[0 1 0],'filled');  %weizuobiao
%  hold on; 


% get 3D coordinates of facial features, using the refU map from image 2D to model 3D
ind = sub2ind([size(refU,1), size(refU,2)], round(ref_XY(:,2)), round(ref_XY(:,1)));
threedee = zeros(numel(ind),3);
tmp = refU(:,:,1);
threedee(:,1) = tmp(ind);
tmp = refU(:,:,2);
threedee(:,2) = tmp(ind);
tmp = refU(:,:,3);
threedee(:,3) = tmp(ind);

% a bit of sanity
indbad = find(max(threedee,[],2)==0);
threedee(indbad,:)=[];
scatter3(threedee(:,1),threedee(:,2),threedee(:,3),40,[1 0 1],'filled');

%% STEP 2 - LOAD QUERY IMAGE, DETECT ITS FACIAL FEATURES
queryI = imread(['1.jpg']);
[newheight, newwidth, dim] = size(queryI);
figure,imshow(queryI); hold on; 
img_bs = detect(queryI, model,0.1);
img_bs = clipboxes(queryI, img_bs);
img_bs = nms_face(img_bs,0.3);
x1 = img_bs.xy(:,1);
y1 = img_bs.xy(:,2);
x2 = img_bs.xy(:,3);
y2 = img_bs.xy(:,4);
img_XY = [(x1+x2)/2,(y1+y2)/2];
img_XY(indbad,:)=[];
x3 = img_XY(:,1);
y3 = img_XY(:,2);
scatter(x3,y3,20,[1,0,1],'filled'); hold on; 
%scatter(x1,y1,20,[1,0,1],'filled'); hold on; 
%scatter(x2,y2,20,[0,1,1],'filled');


%% STEP 3 - COMPUTE POSE USING REFERENCE 3D POINTS + QUERY 2D POINTS, AND RE-RENDER
% Estimate pose
[est_A,est_R,est_T]=doCalib(size(refU,2),size(refU,1),img_XY,threedee,outA,[],[]);
% re-render reference to the same pose as query. Note the change in input and output parameters vs. the previous call. Also note that the model file is not reloaded and a cached version is used instead.
[depth_new, rendered_new,unproject,A, R, T] = renderer(newwidth, newheight, model_name, 1, 1, est_A, est_R, est_T); 
figure, imshow(rendered_new);

map_index=saveobjmesh('unproject1.obj',unproject(:,:,1),unproject(:,:,2),unproject(:,:,3));
save PreCeres_face.mat; 

close all ; clear all ; 

