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
	call :FileLink esp\!fname! "..\%%i"
)

for %%i in (..\esp_at_lib\src\include\esp\*.h) do (
	call :GetFileName fname "%%i"
	call :FileLink esp\!fname! "..\%%i"
)

if not exist cli mkdir cli
for %%i in (..\esp_at_lib\src\cli\*.c) do (
	call :GetFileName fname "%%i"
	call :FileLink cli\!fname! "..\%%i"
)

for %%i in (..\esp_at_lib\src\include\cli\*.h) do (
	call :GetFileName fname "%%i"
	call :FileLink cli\!fname! "..\%%i"
)

if not exist system mkdir system
for %%i in (..\esp_at_lib\src\system\esp_sys_freertos_os.c) do (
	call :GetFileName fname "%%i"
	call :FileLink system\!fname! "..\%%i"
)
call :FileLink esp_sys_port.h ..\esp_at_lib\src\include\system\port\freertos\esp_sys_port.h

call :FileLink system\esp_ll.h "..\..\esp_at_lib\src\include\system\esp_ll.h"
call :FileLink system\esp_sys.h "..\..\esp_at_lib\src\include\system\esp_sys.h"

call :FileLink station_manager.h "..\snippets\include\station_manager.h"
:: call :FileLink station_manager.c "..\snippets\station_manager.c"

if not exist api mkdir api
for %%i in (..\esp_at_lib\src\api\esp_netconn.c) do (
	call :GetFileName fname "%%i"
	call :FileLink api\!fname! "..\%%i"
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

:FileLink
REM --Make a file linkage, could be mklink or copy directly.
	setlocal
	set link=%~1
	set tar=%~2

	:: mklink %link% %tar%

	:: up    link
	:: or
	:: below copy

	set linkpath=%~p1
	set linkname=%~nx1
	cd %linkpath%
	echo copy %tar% %linkname%
	copy %tar% %linkname%
	(
		endlocal & REM -- RETURN VALUES
	)
goto :eof
