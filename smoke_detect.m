function varargout = smoke_detect(varargin)
% SMOKE_DETECT MATLAB code for smoke_detect.fig
%      SMOKE_DETECT, by itself, creates a new SMOKE_DETECT or raises the existing
%      singleton*.
%
%      H = SMOKE_DETECT returns the handle to a new SMOKE_DETECT or the handle to
%      the existing singleton*.
%
%      SMOKE_DETECT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SMOKE_DETECT.M with the given input arguments.
%
%      SMOKE_DETECT('Property','Value',...) creates a new SMOKE_DETECT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before smoke_detect_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to smoke_detect_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help smoke_detect

% Last Modified by GUIDE v2.5 28-May-2014 15:18:57

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @smoke_detect_OpeningFcn, ...
    'gui_OutputFcn',  @smoke_detect_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before smoke_detect is made visible.
function smoke_detect_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to smoke_detect (see VARARGIN)

% Choose default command line output for smoke_detect
handles.output = hObject;

global global_pause global_screenshot global_start_frame
global_pause=0;
global_screenshot=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%全局变量

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes smoke_detect wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = smoke_detect_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit_area_pic_Callback(hObject, eventdata, handles)
% hObject    handle to edit_area_pic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_area_pic as text
%        str2double(get(hObject,'String')) returns contents of edit_area_pic as a double


% --- Executes during object creation, after setting all properties.
function edit_area_pic_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_area_pic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_start.
function pushbutton_start_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global global_pause global_screenshot global_start_frame target_position
if ~isfield(handles, 'FileName')
    return;%return 作用是为了跳出函数
end

attenuation_ratio_str = get(handles.edit_attenuation_ratio, 'String');
if isempty(attenuation_ratio_str)
    return;
end
attenuation_ratio = str2double(attenuation_ratio_str);

area_thresh_str = get(handles.edit_area_thresh, 'String');
if isempty(area_thresh_str)
    return;
end
area_thresh = str2double(area_thresh_str);%首先输入字符，然后字符转双精度数

x_distance_str = get(handles.edit_x_distance, 'String');
if isempty(x_distance_str)
    return;
end
x_distance = str2double(x_distance_str);

y_distance_str = get(handles.edit_y_distance, 'String');
if isempty(y_distance_str)
    return;
end
y_distance = str2double(y_distance_str);


FileName = handles.FileName;

obj = VideoReader(FileName);
nFrames = obj.NumberOfFrames;
FrameRate=obj.FrameRate;
area = zeros(nFrames,1);
area_effect_num = 0;
area_effect = zeros(nFrames,1);
flag = true; % 烟雾起始帧
flag1 = true; % 有效遮蔽面积起始帧
img1=read(obj,1);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k=1;
while 1
    a=global_pause;
    if a==0&&k<=nFrames
   %% for k = 1:5:nFrames
    % 获取每帧图片
    img = read(obj, k);     
    img=img(150:450,100:870);
    % 二值化
    bw = im2bw(img,0.4);
    % 膨胀操作
    se=strel('disk', 2);
    bw=imdilate(bw, se);
    % 去掉小面积的点
    bw1 = bwareaopen(bw,220);
    % 得到小面积的图
    bw = bw - bw1;
       if  k==1 
           bw2=bw;
       end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      if global_screenshot==1
         %t=k/FrameRate;
          imwrite(img,strcat('第',num2str(k),'帧','.jpg'));
           global_screenshot=0;
             end
    
    if k == 1
        % 得到每个亮点的中心位置
        CC = bwconncomp(bw);
        s  = regionprops(CC, 'centroid', 'Area');
        centroids1 = cat(1, s.Centroid);
        % 按第列排序
        [centroids_y, index] = sort(centroids1(:,2));%对纵坐标排序，centroids—y中是新的纵坐标
        centroids_x = centroids1(:,1);
        centroids_x = centroids_x(index);
        centroids = [centroids_x centroids_y];
        centroids=round(centroids);
        % 作差分，分割出每行的点
        d = diff(centroids);
        loc = find(d(:,2) > 10);
        loc(length(loc)+1)  = size(centroids,1);
        % 按行排序
        pts = cell(length(loc),1);%%%%%创建数组，根据靶标行数  pts为每行亮点坐标数组
        pts{1} = centroids(1:loc(1),:);
         x1 = centroids(1:loc(1), :);
            [p_x, index] = sort(x1(:,1));
            p_y = x1(:,2);
            p_y = p_y(index);
            pts{1} = [ p_x, p_y];%%%%%对第一行单独进行排列
        for i = 2: length(loc)
            x = centroids(loc(i-1)+1:loc(i), :);
            [p_x, index] = sort(x(:,1));
            p_y = x(:,2);
            p_y = p_y(index);
            pts{i} = [ p_x, p_y];
            pts{i}=round(pts{i});
        end
          
        pts{3}(10,:)=[];
        %取出每个点的灰度值
       grey_value=cell(length(pts),1);
       for i=1:length(pts)
           x=pts{i};
           for j=1:length(x)
           grey_value{i}(j)=img(x(j,2),x(j,1));
           end
       end
        % 计算相邻亮点的中心位置，本方法选取靶标右侧中点位置附近的点作为背景
        cent = cell(length(loc),1);    %%%%%cent为每行亮点中心位置数组
        for i = 1: length(pts)
            x = pts{i};
            cent{i} = (x(1:end-1,:)+x(2:end,:))/2;
            g=length(pts{i});
            %边缘处理，将最右边靶标的中点位置取成上一个中点值
            cent{i}(g,1)=cent{i}(g-1,1);aa=cent{i}(g-1,1);
            cent{i}(g,2)=cent{i}(g-1,2);bb=cent{i}(g-1,2);
            AA=cent{i}(:,1);
            BB=AA; %-5意思指距离靶标的距离更加接近5个像素点
            cent{i}(:,1)=BB;
            cent{i}(g,1)=aa;
            cent{i}(g,2)=bb;
        end
      
       
%         if isempty(cent{1})
%             k = k+1;
%             continue;
%         end
        % 计算亮点位置与相邻亮点中心位置的灰度差，也就是分母部分
        den=zeros(length(loc),1);
      for i = 1: length(cent)
            x1 = round(pts{i});
            x2 = round(cent{i});
            tmp = zeros(size(x1,1),1);%tmp为每行的分母矩阵
            for j = 1: size(x1,1)
                tmp(j) = img(x1(j,2),x1(j,1)) - img(x2(j,2),x2(j,1));  
            end
             tmp=max(tmp);den(i)=tmp;%分母取每行中的最大值
            %den{i} = tmp;
      end
        aver_d=max(den);
    end
    % 计算有烟雾时，上述两位置的灰度差，也就是分子部分
    if k > 2
        num = cell(length(loc),1);                       %%%%%%num为透过率分子部分数组
        for i = 1: length(cent)
            x1 = round(pts{i});
            x2 = round(cent{i});
            tmp = zeros(size(x1,1),1);
            for j = 1: size(x1,1)
                tmp(j) = img(x1(j,2),x1(j,1)) - img(x2(j,2),x2(j,1));
            end
            num{i} = tmp;
        end
        % 判断是否为烟雾  标准1：是否满足透过率标准
        c=1;
        target_position=zeros(100,2);%规定一个参数使得记录满足透过率的点的坐标的技术标准
        for i = 1: length(num)% 对行进行排列
            for j = 1: length(num{i})
                if num{i}(j)/aver_d < 1-attenuation_ratio
                    area(k) = area(k) + 1;
                    x_label=pts{i}(j,1);y_label=pts{i}(j,2);
                    target_position(c,:)=[x_label,y_label];
                    c=c+1;                   
                    %set(gcf, 'CurrentAxes', handles.axes_process);
                    %axes(handles.axe_process);
                    %输出满足点的坐标
                end
            end
        end
        target_position(c:end,:)=[];
        % 判断是否为有效面积 标准2：是否满足遮蔽面积阈值
        if  area(k)*x_distance*y_distance >=  area_thresh 
            area_effect_num = area_effect_num + 1;
            area_effect(area_effect_num) = area(k);
        end
        
        if flag && area(k) > 1
            smoke_time = k;
            flag = false;
            global_start_frame=k;
        end
        
        if flag1 && area(k)*x_distance*y_distance >= area_thresh 
            effect_time = k;
            flag1 = false;
        end
    end
    
    set(gcf, 'CurrentAxes', handles.axes_origin);
    imshow(img);hold on;
    %plot(390,194,'ro','MarkerSize',12,'LineWidth',2);  //标记样本目标点
    set(gcf, 'CurrentAxes', handles.axes_process);
    imshow(bw2);%hold on; 
   % if sum(target_position)~=0
    % plot(target_position(:,1),target_position(:,2),'ro','MarkerSize',10,'LineWidth',2);
   % end
 
    set(handles.text_title,'String',['Framenumber:',num2str(k)]);
    set(handles.edit_area_pic, 'String', num2str(area(k)));
    set(handles.edit_area_world, 'String', num2str(area(k)*x_distance*y_distance));

    pause(0.000001)
    k=k+1;%%%%递增
    end
    if a==1
       pause(1);
            if global_screenshot==1
            imwrite(img,strcat('第',num2str(k),'帧','.jpg'));
           global_screenshot=0;
             end
       
        continue;
    end
    if k>nFrames
    break;
    end
    
   
end
       

   if area_effect_num < 1
    msgbox('无有效遮蔽时间！');
    else
    area_effect = area_effect(area_effect~=0);
    area_effect_avg = mean(area_effect);
    set(handles.edit_shade_area, 'String', num2str(area_effect_avg*x_distance*y_distance));
    set(handles.edit_shade_time, 'String', num2str(area_effect_num/FrameRate));%%%%%%5帧需改参数的地方
    set(handles.edit_smoke_time, 'String', num2str((effect_time - smoke_time)/FrameRate));
   end
    handles.area = area;
    handles.nFrames=nFrames;
    handles.FrameRate=FrameRate;
    guidata(hObject, handles);
  


 




% --------------------------------------------------------------------
function File_Callback(hObject, eventdata, handles)
% hObject    handle to File (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --------------------------------------------------------------------
function Exit_Callback(hObject, eventdata, handles)
% hObject    handle to Exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close;

% --------------------------------------------------------------------
function Open_Callback(hObject, eventdata, handles)
% hObject    handle to Open (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile( ...
    {'*.avi','.avi';
    '*.*',  'All Files (*.*)'}, ...
    '选择一个视频文件');
if isempty(filename)
    msgbox('未打开视频！');
    return;
end
FileName = [pathname,filename];
handles.FileName = FileName;
guidata(hObject, handles);


function edit_area_world_Callback(hObject, eventdata, handles)
% hObject    handle to edit_area_world (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_area_world as text
%        str2double(get(hObject,'String')) returns contents of edit_area_world as a double


% --- Executes during object creation, after setting all properties.
function edit_area_world_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_area_world (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_data.
function pushbutton_data_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global global_start_frame
if ~isfield(handles, 'area')
    return;
end
h = findobj('Tag','data');
if ~isempty(h)
    close(h);
end
nFrames=handles.nFrames;
FrameRate=handles.FrameRate;

x_distance_str = get(handles.edit_x_distance, 'String');
if isempty(x_distance_str)
    return;
end
x_distance = str2double(x_distance_str);

y_distance_str = get(handles.edit_y_distance, 'String');
if isempty(y_distance_str)
    return;
end
y_distance = str2double(y_distance_str);

figure('Tag','data');
start_time=global_start_frame/FrameRate;
%x_time=0.04:1/FrameRate:(nFrames-1)/FrameRate;%改5帧需要更改的地方
x1_time=start_time:1/FrameRate:(nFrames-1)/FrameRate;
%y=handles.area(fix(x_time*FrameRate));
y1=handles.area(fix(x1_time*FrameRate));
%plot(x_time,y,'b')
%title('Smoke area in picture');
%xlabel('time/s');
%ylabel('area/m^2');
%set(gca,'xlim',[0 55],'xtick',[0:1:50],'fontsize',10);
%subplot(212)
plot(x1_time,y1*x_distance*y_distance,'r')
title('Smoke area in field');
xlabel('time/s');
ylabel('area/m^2');
%set(gca,'xlim',[0 40],'xtick',[0:1:50],'fontsize',10);
A=zeros(length(x1_time),2);
A(:,1)=x1_time;
A(:,2)=y1*x_distance*y_distance;
A
dlmwrite('smoke_area.txt',A,'precision','%.2f','delimiter','\t');




function edit_shade_area_Callback(hObject, eventdata, handles)
% hObject    handle to edit_shade_area (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_shade_area as text
%        str2double(get(hObject,'String')) returns contents of edit_shade_area as a double


% --- Executes during object creation, after setting all properties.
function edit_shade_area_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_shade_area (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_shade_time_Callback(hObject, eventdata, handles)
% hObject    handle to edit_shade_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_shade_time as text
%        str2double(get(hObject,'String')) returns contents of edit_shade_time as a double


% --- Executes during object creation, after setting all properties.
function edit_shade_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_shade_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_smoke_time_Callback(hObject, eventdata, handles)
% hObject    handle to edit_smoke_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_smoke_time as text
%        str2double(get(hObject,'String')) returns contents of edit_smoke_time as a double


% --- Executes during object creation, after setting all properties.
function edit_smoke_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_smoke_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_area_thresh_Callback(hObject, eventdata, handles)
% hObject    handle to edit_area_thresh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_area_thresh as text
%        str2double(get(hObject,'String')) returns contents of edit_area_thresh as a double


% --- Executes during object creation, after setting all properties.
function edit_area_thresh_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_area_thresh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_attenuation_ratio_Callback(hObject, eventdata, handles)
% hObject    handle to edit_attenuation_ratio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_attenuation_ratio as text
%        str2double(get(hObject,'String')) returns contents of edit_attenuation_ratio as a double


% --- Executes during object creation, after setting all properties.
function edit_attenuation_ratio_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_attenuation_ratio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_y_distance_Callback(hObject, eventdata, handles)
% hObject    handle to edit_y_distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_y_distance as text
%        str2double(get(hObject,'String')) returns contents of edit_y_distance as a double


% --- Executes during object creation, after setting all properties.
function edit_y_distance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_y_distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pause.
function pause_Callback(hObject, eventdata, handles)
% hObject    handle to pause (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global global_pause
name=get(handles.pause,'string');
switch name
    case 'Pause'
        set(handles.pause,'string','Continue');
        global_pause=1;
    case 'Continue'
        set(handles.pause,'string','Pause');
        global_pause=0;
end


% --- Executes on button press in screenshot.
function screenshot_Callback(hObject, eventdata, handles)
% hObject    handle to screenshot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global global_screenshot
global_screenshot=1;
