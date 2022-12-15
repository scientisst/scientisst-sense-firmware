@echo off

rem Get current script's absolute path
set SCRIPT_PATH=%CD%
echo %SCRIPT_PATH%

set IDF_PATH=%SCRIPT_PATH%\deps\esp-idf\
set IDF_TOOLS_PATH=%SCRIPT_PATH%\esp-idf-tools\

rem If an argument is not supplied (don't skip the export part)
if "%1" == "" (
    rem Export idf
    call %IDF_PATH%\export.bat

    rem Create command to open serial monitor with ease
    doskey idf_monitor=idf.py monitor -p /dev/ttyUSB0
)

rem Check for arguments
if not "%1" == "" (
    set POSITIONAL_ARGS=()
    :arg_loop
    if "%1" == "" goto arg_loop_end
    if "%1" == "--install" set INSTALL=1& echo % INSTALL% & shift  & goto arg_loop
    if "%1" == "-*" goto unknown_arg
    if "%1" == "--*" goto unknown_arg
    set POSITIONAL_ARGS=%POSITIONAL_ARGS% %1
    shift
    goto arg_loop

    :arg_loop_end
    if "%INSTALL%" equ "1" (
        %SCRIPT_PATH%\deps\esp-idf\install.bat
    ) else (
)
    :unknown_arg
    echo Unknown option %1
    rem exit /b 1
)
