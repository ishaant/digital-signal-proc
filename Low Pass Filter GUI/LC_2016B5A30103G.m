function varargout = LC_2016B5A30103G(varargin)
% LC_2016B5A30103G MATLAB code for LC_2016B5A30103G.fig
%      LC_2016B5A30103G, by itself, creates a new LC_2016B5A30103G or raises the existing
%      singleton*.
%
%      H = LC_2016B5A30103G returns the handle to a new LC_2016B5A30103G or the handle to
%      the existing singleton*.
%
%      LC_2016B5A30103G('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LC_2016B5A30103G.M with the given input arguments.
%
%      LC_2016B5A30103G('Property','Value',...) creates a new LC_2016B5A30103G or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LC_2016B5A30103G_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LC_2016B5A30103G_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LC_2016B5A30103G

% Last Modified by GUIDE v2.5 10-Nov-2019 15:39:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LC_2016B5A30103G_OpeningFcn, ...
                   'gui_OutputFcn',  @LC_2016B5A30103G_OutputFcn, ...
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


% --- Executes just before LC_2016B5A30103G is made visible.
function LC_2016B5A30103G_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LC_2016B5A30103G (see VARARGIN)

% Choose default command line output for LC_2016B5A30103G
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LC_2016B5A30103G wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LC_2016B5A30103G_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_f1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1_f1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num1 = get(handles.edit1_f1, 'String');
num1 = str2num(num1);
handles.num1 = num1;
guidata(hObject, handles);

% Hints: get(hObject,'String') returns contents of edit1_f1 as text
%        str2double(get(hObject,'String')) returns contents of edit1_f1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_f1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1_f1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_f2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2_f2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num2 = get(handles.edit2_f2, 'String');
num2 = str2num(num2);
handles.num2 = num2;
guidata(hObject, handles);

% Hints: get(hObject,'String') returns contents of edit2_f2 as text
%        str2double(get(hObject,'String')) returns contents of edit2_f2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_f2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2_f2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_f3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3_f3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num3 = get(handles.edit3_f3, 'String');
num3 = str2num(num3);
handles.num3 = num3;
guidata(hObject, handles);

% Hints: get(hObject,'String') returns contents of edit3_f3 as text
%        str2double(get(hObject,'String')) returns contents of edit3_f3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_f3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3_f3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in radiobutton_out.
function radiobutton_out_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton_out (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
N = [handles.num1 handles.num2 handles.num3];
fs=10000;
t=0:1/fs:0.01;
sin1=sin(2*pi*N(1)*t);
sin2=sin(2*pi*N(2)*t);
sin3=sin(2*pi*N(3)*t);
inp_sig=sin1+sin2+sin3;

axes(handles.axes1)

Fs = 10000;  % Sampling Frequency

N    = 22;       % Order
Fc   = 2800;     % Cutoff Frequency
flag = 'scale';  % Sampling Flag

% Create the window vector for the design algorithm.
win = blackman(N+1);

b  = fir1(N, Fc/(Fs/2), 'low', win, flag);


filtered_sig=filter(b,1,inp_sig);
plot(t,filtered_sig,'r')
title('Filtered')
xlabel('Time Index (n)')
ylabel('Amplitude')
grid on

N=100
axes(handles.axes2)
filtered_sig_f=abs(fft(filtered_sig,N));
filtered_sig_f=2*filtered_sig_f/N;
f_axis2=(0:(N-1))*Fs/N;
stem(f_axis2(1:N/2),filtered_sig_f(1:N/2))
title('Spectrum-Filtered Signal')
xlabel('Frequency in Hz')
ylabel('Amplitude')

% Hint: get(hObject,'Value') returns toggle state of radiobutton_out


% --- Executes on button press in radiobutton_filt.
function radiobutton_filt_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton_filt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Fs = 10000;  % Sampling Frequency

N    = 22;       % Order
Fc   = 2800;     % Cutoff Frequency
flag = 'scale';  % Sampling Flag

% Create the window vector for the design algorithm.
win = blackman(N+1);

% Calculate the coefficients using the FIR1 function.
b  = fir1(N, Fc/(Fs/2), 'low', win, flag);
xa=1:length(b);
xa=xa-1;
axes(handles.axes1)
stem(0:length(b)-1,b);
title('Filter Coefficients')
xlabel('n')
ylabel('h(n)')
xlim([0 22])



axes(handles.axes2)
[h,w]=freqz(b,1,2048);
magn_h=20*log10(abs(h));
plot(Fs*w/(2*pi),magn_h)
title('Filter Response')
xlabel('Frequency in Hz')
ylabel('Magnitude(in dB)')
grid on




% Hint: get(hObject,'Value') returns toggle state of radiobutton_filt


% --- Executes on button press in radiobutton_inp.
function radiobutton_inp_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton_inp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
N = [handles.num1 handles.num2 handles.num3];
fs=10000;
t=0:1/fs:0.01;
sin1=sin(2*pi*N(1)*t);
sin2=sin(2*pi*N(2)*t);
sin3=sin(2*pi*N(3)*t);
inp_sig=sin1+sin2+sin3;
axes(handles.axes1)
plot(t,inp_sig);
xlim([0 0.01])
ylim([-3 3])
title('Time Domain View'); xlabel('Time'); ylabel('Amplitude (V)');

Fs = 10000;  % Sampling Frequency

N    = 100;       % Order
Fc   = 2800;     % Cutoff Frequency
flag = 'scale';  % Sampling Flag

axes(handles.axes2)
inp_sig_f=abs(fft(inp_sig,N));
inp_sig_f=2*inp_sig_f/N;
f_axis2=(0:(N-1))*Fs/N;
stem(f_axis2(1:N/2),inp_sig_f(1:N/2))
title('Spectrum-Input Signal')
xlabel('Frequency in Hz')
ylabel('Amplitude')

% Hint: get(hObject,'Value') returns toggle state of radiobutton_inp
