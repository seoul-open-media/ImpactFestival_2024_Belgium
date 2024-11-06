@echo pd_dsp
@echo off

set CURRENT_PATH=%cd%
set "CURRENT_DRIVE=%CURRENT_PATH:~,2%"

%CURRENT_DRIVE%
cd "%CURRENT_PATH%"

set PD_PATCH=""
set PD_BIN=%PROGRAMFILES(X86)%\pd-0.49-0\bin
set PD_EXTRA=%CURRENT_PATH%

set PD_AUDIO=-channels 2 -r 44100 -audiobuf 120 -blocksize 128
set PD_OPTIONS=-font 10 -midiindev 1 -midioutdev 3 -noprefs
rem set PD_PATH=%PD_EXTRA%/help;%PD_EXTRA%/abs
set PD_PATH=help;abs
set PD_LIB=iemlib

@echo starting pd_dsp ...
rem start "" /REALTIME %PD_BIN%\pd.exe %PD_AUDIO% %PD_OPTIONS% %PD_PATCH%

start "" "%PD_BIN%\pd.exe" %PD_AUDIO% -path "%PD_PATH%" -lib %PD_LIB% -verbose