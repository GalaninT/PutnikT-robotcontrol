% Adams / MATLAB Interface - Release 2020.0.0
global ADAMS_sysdir; % used by setup_rtw_for_adams.m
global ADAMS_host; % used by start_adams_daemon.m
machine=computer;
datestr(now)
if strcmp(machine, 'GLNXA64')
   arch = 'linux64';
elseif strcmp(machine, 'PCWIN64')
   arch = 'win64';
else
   disp( '%%% Error : Platform unknown or unsupported by Adams Controls.' ) ;
   arch = 'unknown_or_unsupported';
   return
end
   [flag, topdir]=system('adams2020 -top');
if flag == 0
  temp_str=strcat(topdir, '/controls/', arch);
  addpath(temp_str)
  temp_str=strcat(topdir, '/controls/', 'matlab');
  addpath(temp_str)
  temp_str=strcat(topdir, '/controls/', 'utils');
  addpath(temp_str)
  ADAMS_sysdir = strcat(topdir, '');
else
  addpath( 'C:\PROGRA~1\MSC~1.SOF\Adams\2020_7~1\controls/win64' ) ;
  addpath( 'C:\PROGRA~1\MSC~1.SOF\Adams\2020_7~1\controls/matlab' ) ;
  addpath( 'C:\PROGRA~1\MSC~1.SOF\Adams\2020_7~1\controls/utils' ) ;
  ADAMS_sysdir = 'C:\PROGRA~1\MSC~1.SOF\Adams\2020_7~1\' ;
end
ADAMS_exec = '' ;
ADAMS_host = 'MSI' ;
ADAMS_cwd ='C:\Users\wfism'  ;
ADAMS_prefix = 'Controls_Plant_2' ;
ADAMS_static = 'no' ;
ADAMS_solver_type = 'C++' ;
ADAMS_version = '2020' ;
if exist([ADAMS_prefix,'.adm']) == 0
   disp( ' ' ) ;
   disp( '%%% Warning : missing Adams plant model file(.adm) for Co-simulation or Function Evaluation.' ) ;
   disp( '%%% If necessary, please re-export model files or copy the exported plant model files into the' ) ;
   disp( '%%% working directory.  You may disregard this warning if the Co-simulation/Function Evaluation' ) ;
   disp( '%%% is TCP/IP-based (running Adams on another machine), or if setting up MATLAB/Real-Time Workshop' ) ;
   disp( '%%% for generation of an External System Library.' ) ;
   disp( ' ' ) ;
end
ADAMS_init = '' ;
ADAMS_inputs  = 'hipFLTorque!hipFRTorque!hipMLTorque!hipMRTorque!kneeFLTorque!kneeFRTorque!kneeMLTorque!kneeMRTorque!steerFLTorque!steerFRTorque!steerMLTorque!steerMRTorque!wheelFLTorque!wheelFRTorque!wheelMLTorque!wheelMRTorque' ;
ADAMS_outputs = 'bodyAcceleration_X!bodyAcceleration_Y!bodyAcceleration_Z!bodyAngle_Pitch!bodyAngle_Roll!bodyAngle_Yaw!bodyAngVelocity_X!bodyAngVelocity_Y!bodyAngVelocity_Z!bodyCoord_X!bodyCoord_Y!bodyCoord_Z!bodyVelocity_X!bodyVelocity_Y!bodyVelocity_Z!hipFLAcceleration!hipFLAngle!hipFLSpeed!hipFRAcceleration!hipFRAngle!hipFRSpeed!hipMLAcceleration!hipMLAngle!hipMLSpeed!hipMRAcceleration!hipMRAngle!hipMRSpeed!kneeFLAcceleration!kneeFLAngle!kneeFLSpeed!kneeFRAcceleration!kneeFRAngle!kneeFRSpeed!kneeMLAcceleration!kneeMLAngle!kneeMLSpeed!kneeMRAcceleration!kneeMRAngle!kneeMRSpeed!steerFLAcceleration!steerFLAngle!steerFLSpeed!steerFRAcceleration!steerFRAngle!steerFRSpeed!steerMLAcceleration!steerMLAngle!steerMLSpeed!steerMRAcceleration!steerMRAngle!steerMRSpeed!wheelFLAcceleration!wheelFLAngle!wheelFLForce_X!wheelFLForce_Y!wheelFLForce_Z!wheelFLSpeed!wheelFRAcceleration!wheelFRAngle!wheelFRForce_X!wheelFRForce_Y!wheelFRForce_Z!wheelFRSpeed!wheelMLAcceleration!wheelMLAngle!wheelMLForce_X!wheelMLForce_Y!wheelMLForce_Z!wheelMLSpeed!wheelMRAcceleration!wheelMRAngle!wheelMRForce_X!wheelMRForce_Y!wheelMRForce_Z!wheelMRSpeed' ;
ADAMS_pinput = 'Controls_Plant_2.ctrl_pinput' ;
ADAMS_poutput = 'Controls_Plant_2.ctrl_poutput' ;
ADAMS_uy_ids  = [
                   100
                   108
                   96
                   104
                   99
                   107
                   95
                   103
                   98
                   106
                   94
                   102
                   97
                   105
                   93
                   101
                   165
                   166
                   167
                   113
                   114
                   112
                   168
                   169
                   170
                   109
                   110
                   111
                   162
                   163
                   164
                   156
                   125
                   145
                   150
                   115
                   139
                   159
                   129
                   138
                   153
                   120
                   142
                   157
                   124
                   146
                   151
                   116
                   140
                   160
                   128
                   148
                   154
                   119
                   143
                   158
                   123
                   147
                   152
                   117
                   141
                   161
                   127
                   149
                   155
                   121
                   144
                   135
                   186
                   175
                   178
                   181
                   126
                   134
                   187
                   171
                   172
                   173
                   118
                   137
                   188
                   176
                   179
                   182
                   130
                   136
                   189
                   174
                   177
                   180
                   122
                ] ;
ADAMS_mode   = 'non-linear' ;
tmp_in  = decode( ADAMS_inputs  ) ;
tmp_out = decode( ADAMS_outputs ) ;
disp( ' ' ) ;
disp( '%%% INFO : ADAMS plant actuators names :' ) ;
disp( [int2str([1:size(tmp_in,1)]'),blanks(size(tmp_in,1))',tmp_in] ) ;
disp( '%%% INFO : ADAMS plant sensors   names :' ) ;
disp( [int2str([1:size(tmp_out,1)]'),blanks(size(tmp_out,1))',tmp_out] ) ;
disp( ' ' ) ;
clear tmp_in tmp_out ;
% Adams / MATLAB Interface - Release 2020.0.0
