function varargout = convgui1(varargin)
% CONVGUI1 MATLAB code for convgui1.fig
%      CONVGUI1, by itself, creates a new CONVGUI1 or raises the existing
%      singleton*.
%
%      H = CONVGUI1 returns the handle to a new CONVGUI1 or the handle to
%      the existing singleton*.
%
%      CONVGUI1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONVGUI1.M with the given input arguments.
%
%      CONVGUI1('Property','Value',...) creates a new CONVGUI1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before convgui1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to convgui1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help convgui1

% Last Modified by GUIDE v2.5 02-Oct-2023 22:54:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @convgui1_OpeningFcn, ...
    'gui_OutputFcn',  @convgui1_OutputFcn, ...
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


% --- Executes just before convgui1 is made visible.
function convgui1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to convgui1 (see VARARGIN)

% Choose default command line output for convgui1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes convgui1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = convgui1_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x=eval(get(handles.edit1,'String'));
nx=0:length(x)-1;
axes(handles.axes1);
stem(nx,x);

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
y=eval(get(handles.edit2,'String'));
ny=0:length(y)-1;
axes(handles.axes2);
stem(ny,y);

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x=eval(get(handles.edit1,'String'));
nx=0:length(x)-1;
y=eval(get(handles.edit2,'String'));
ny=0:length(y)-1;
[z,nz]=convolution(x,nx,y,ny);
axes(handles.axes3);
stem(nz,z);


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes1);
cla reset;
axes(handles.axes2);
cla reset;
axes(handles.axes3);
cla reset;
axes(handles.axes4);
cla reset;
% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x=eval(get(handles.edit1,'String'));
nx=0:length(x)-1;
y=eval(get(handles.edit2,'String'));
ny=0:length(y)-1;
N=eval(get(handles.edit3,'String'));
z=circonvtim(x,y,N);
axes(handles.axes4);
stem(0:length(z)-1,z);

function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x=eval(get(handles.edit1,'String'));
h=eval(get(handles.edit2,'String'));
lx =length(x); 
lh=length(h);lmax=max(lx,lh);
 if lx > lh nx =0; nh = lx - lh ;
 elseif lx< lh nh =0; nx = lh -lx;
 else nx =0; nh =0;%将序列对齐%
 end 
lt= lmax;
 u =[zeros(1,lt),x, zeros(1,nx),zeros(1,lt)];
t1=(- lt +1:2*lt) ;
 h =[zeros(1,2*lt),h,zeros(1,nh)];
 hf =fliplr(h);%翻褶%
 N =length(hf);
 y = zeros(1,3*lt);
 dt = 1;
for k=0:2*lt
 p =[zeros(1,k),hf(1:N-k)];
 y1=u.*p ;
 yk=sum(y1);
 y(k +lt+1)=yk; 
 axes(handles.axes1);stem(t1,u);
 axis([-lt,2*lt,min(u), max(u)]); hold on ;ylabel('x(n)');title('线性卷积的演示');
 axes(handles.axes2);stem(t1,p); axis([-lt ,2*lt,min(p), max(p)]);ylabel('h(k-n)');
 axes(handles.axes3);stem(t1,y1);axis([-lt ,2*lt,min(y1),max(y1)+eps]);
 ylabel('s =u.*h(k-n)');
 axes(handles.axes4); stem(k,yk);
 axis([-lt,2*lt, floor(min(y)+eps), ceil(max(y+eps))]); hold on ;
 ylabel ('y(k)=sum(s)');
 pause(1);drawnow
    frame = getframe(gcf);
    im = frame2im(frame);
    [I,map] = rgb2ind(im,128);
    if k>0
        imwrite(I,map,'lc.gif','WriteMode','append','DelayTime',dt);
    else
        imwrite(I,map,'lc.gif','LoopCount',Inf,'DelayTime',dt);
    end
end

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x=eval(get(handles.edit1,'String'));
h=eval(get(handles.edit2,'String'));
N=eval(get(handles.edit3,'String'));
n=0:N-1;
x=[x,zeros(1,N-length(x))];
h=[h,zeros(1,N-length(h))];
h1=h(mod(-n,N)+1);
dt = 1;
for m = 0:N-1
    h2=cirshftt(h1,m,N);
    h3=x.*h2;
    hk=sum(h3);
    h(m+1)=hk;
 axes(handles.axes1); stem(n,x);axis([0,N-1,min(x),max(x)]); hold on ;ylabel('x(n)');
 title('圆周卷积的演示');
axes(handles.axes2);stem(n,h2);axis([0,N-1,min(h2),max(h2)]);ylabel('h(k-n)');
 axes(handles.axes3);stem(n,h3);axis([0,N-1,min(h3),max(h3)+eps]);
 ylabel('s =u.*h(k-n)');
 axes(handles.axes4); stem(m,hk);axis([0,N-1,min(h),max(h)]); hold on ;
 ylabel ('y(k)=sum(s)');
 pause(1);drawnow
    frame = getframe(gcf);
    im = frame2im(frame);
    [I,map] = rgb2ind(im,128);
    if m>0
        imwrite(I,map,'cc.gif','WriteMode','append','DelayTime',dt);
    else
        imwrite(I,map,'cc.gif','LoopCount',Inf,'DelayTime',dt);
    end
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x=eval(get(handles.edit1,'String'));
h=eval(get(handles.edit2,'String'));
N=eval(get(handles.edit3,'String'));
n=0:N-1;
x1=[x,zeros(1,N-length(x))];
x2=[h,zeros(1,N-length(h))];
Xk1=DFTmat(x1);
Xk2=DFTmat(x2);
real_Xk1=real(Xk1);imag_Xk1=imag(Xk1);
real_Xk2=real(Xk2);imag_Xk2=imag(Xk2);
axes(handles.axes1);plot(n,real_Xk1,'bo',n,imag_Xk1,'r*');ylabel('X_1(k)');
title("蓝紫色为实部，红色为虚部");
axes(handles.axes2);plot(n,real_Xk2,'bo',n,imag_Xk2,'r*');ylabel('X_2(k)')
Yk=Xk1.*Xk2;
real_Yk=real(Yk);imag_Yk=imag(Yk);
axes(handles.axes3);plot(n,real_Yk,'bo',n,imag_Yk,'r*');ylabel('X_1(k)X_2(k)')
n=0:N-1;k=n;nk=n'*k;
WN=exp(j*2*pi/N);Wnk=WN.^nk;
yn=Yk*Wnk/N;
axes(handles.axes4);stem(n,yn);ylabel('x_1(n)*x_2(n)');
