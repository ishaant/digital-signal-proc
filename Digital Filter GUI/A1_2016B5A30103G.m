function varargout = A1_2016B5A30103G(varargin)
% A1_2016B5A30103G MATLAB code for A1_2016B5A30103G.fig
%      A1_2016B5A30103G, by itself, creates a new A1_2016B5A30103G or raises the existing
%      singleton*.
%
%      H = A1_2016B5A30103G returns the handle to a new A1_2016B5A30103G or the handle to
%      the existing singleton*.
%
%      A1_2016B5A30103G('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in A1_2016B5A30103G.M with the given input arguments.
%
%      A1_2016B5A30103G('Property','Value',...) creates a new A1_2016B5A30103G or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before A1_2016B5A30103G_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to A1_2016B5A30103G_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help A1_2016B5A30103G

% Last Modified by GUIDE v2.5 25-Nov-2019 15:26:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @A1_2016B5A30103G_OpeningFcn, ...
                   'gui_OutputFcn',  @A1_2016B5A30103G_OutputFcn, ...
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


% --- Executes just before A1_2016B5A30103G is made visible.
function A1_2016B5A30103G_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to A1_2016B5A30103G (see VARARGIN)

% Choose default command line output for A1_2016B5A30103G
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes A1_2016B5A30103G wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = A1_2016B5A30103G_OutputFcn(hObject, eventdata, handles) 
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
denom = get(handles.edit2_f2, 'String');
denom = str2num(denom);
handles.denom = denom;
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



function Fs_Callback(hObject, eventdata, handles)
% hObject    handle to Fs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Fs = get(handles.Fs, 'String');
fs = str2num(Fs);
handles.fs = fs;
guidata(hObject, handles);

% Hints: get(hObject,'String') returns contents of Fs as text
%        str2double(get(hObject,'String')) returns contents of Fs as a double


% --- Executes during object creation, after setting all properties.
function Fs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Fs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in radiobutton_inp.
function radiobutton_inp_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton_inp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.checkbox1,'Value',0);


freq=get(handles.slider1,'Value');
fs=handles.fs;
ts=1/fs;
t=0:ts:1;
inp_sig=sin(2*pi*freq*t);
  
 

  inp_fft= fft(inp_sig);N = 2*floor((length(inp_fft))/2);
  axes(handles.axes1)

inp_sig_f=abs(fft(inp_sig,N));
inp_sig_f=2*inp_sig_f/N;
f_axis2=(0:(N-1))*fs/N;
stem(f_axis2(1:N/2),inp_sig_f(1:N/2))
ylim([0 1])
xlim([0 fs/2])
title('Spectrum-Input Signal')
xlabel('Frequency in Hz')
ylabel('Amplitude')

%     
 filtered_sig=filter(handles.num1, handles.denom, inp_sig);
axes(handles.axes2)
filtered_sig_f=abs(fft(filtered_sig));N = 2*floor((length(inp_fft))/2);
filtered_sig_f=2*filtered_sig_f/N;
f_axis2=(0:(N-1))*fs/N;
stem(f_axis2(1:N/2),filtered_sig_f(1:N/2))
ylim([0 1])

xlim([0 fs/2])
title('Spectrum-Filtered Signal')
xlabel('Frequency in Hz')
ylabel('Amplitude')


% Hint: get(hObject,'Value') returns toggle state of radiobutton_inp


% --- Executes on button press in radiobutton_filt.
function radiobutton_filt_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton_filt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
N = handles.num1;
D=handles.denom;
fs=handles.fs;

val=get(handles.checkbox1,'Value');

axes(handles.axes1)
[N1,w]=freqz(N,D,'whole');
% N1=N1/max(abs(N1));
N1_mag=(abs(N1));
L=length(N1);
xax=fs/2*w/pi;
    if (val)
        
        plot(xax(1:L/2),10*log10(N1_mag(1:L/2)));
        ylabel('Magnitude (in dB)')
        ylim([-80 0.5])
        
    else
        plot(fs/2*w(1:L/2)/pi,(N1_mag(1:L/2)));
        ylabel('Magnitude (Linear)')
    end
    
    xlim([0 fs/2])
    title('Magnitude response'); xlabel('Frequency (in Hz)');
    grid on

axes(handles.axes2)

yax=angle(N1)/pi;
 plot(xax(1:L/2),yax(1:L/2));
 xlim([0 fs/2])
title('Phase Response'); xlabel('Frequency (in Hz)'); ylabel('Phase(x pi rad/s)');
grid on
 
 


% Hint: get(hObject,'Value') returns toggle state of radiobutton_filt

% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

val=get(hObject,'Value');

set(handles.radiobutton_inp,'Value',0);
set(handles.radiobutton_filt,'Value',1);

radiobutton_filt_Callback(hObject, eventdata, handles)
 





% --- Executes on button press in push_enter.
function push_enter_Callback(hObject, eventdata, handles)
% hObject    handle to push_enter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fs= handles.fs;
% fs=str2num(fs);
set(handles.slider1,'Min',0);
set(handles.slider1,'Value',0);

inp_max=floor(fs/2);
set(handles.slider1,'Max',inp_max);
myString = sprintf('Max: %d Hz', inp_max);
set(handles.FsMax, 'String', myString);
set(handles.text7, 'String', 'Min: 0 Hz');


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
InputFrequency= get(hObject,'Value');

display(InputFrequency);

set(handles.text5,'String',['Input Frequency: ' num2str(InputFrequency)]);

handles.inputfreq=InputFrequency;

set(handles.radiobutton_inp,'Value',1);
set(handles.radiobutton_filt,'Value',0);

radiobutton_inp_Callback(hObject, eventdata, handles)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end




 
% Hint: get(hObject,'Value') returns toggle state of checkbox1
