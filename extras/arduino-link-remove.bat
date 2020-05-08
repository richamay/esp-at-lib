@echo off
setlocal EnableExtensions
setlocal EnableDelayedExpansion

:: Change current directory for 'Run As Admin'
cd %~dp0

setlocal

cd ..\src
if not exist esp mkdir esp
for %%i in (..\esp_at_lib\src\esp\*.c) do (
	call :GetFileName fname "%%i"
	del esp\!fname!
)

for %%i in (..\esp_at_lib\src\include\esp\*.h) do (
	call :GetFileName fname "%%i"
	del esp\!fname!
)

if not exist cli mkdir cli
for %%i in (..\esp_at_lib\src\cli\*.c) do (
	call :GetFileName fname "%%i"
	del cli\!fname!
)

for %%i in (..\esp_at_lib\src\include\cli\*.h) do (
	call :GetFileName fname "%%i"
	del cli\!fname!
)

for %%i in (..\esp_at_lib\src\system\esp_sys_*_os.c) do (
	call :GetFileName fname "%%i"
	del system\!fname!
)
del esp_sys_port.h


del system\esp_ll.h
del system\esp_sys.h
del system\esp_sys_*_os.h

del station_manager.h
:- del station_manager.c

for %%i in (..\esp_at_lib\src\api\esp_netconn.c) do (
	call :GetFileName fname "%%i"
	del api\!fname!
)

goto :eof

:GetFileName
REM --Get the file name in the path
	setlocal
	set filename=%~nx2
	(
		endlocal & REM -- RETURN VALUES
		if "%~1" neq "" (
			set %~1=%filename%
		)
	)
goto :eof
