@echo off
setlocal
title 小车控制台 · ClawPi
cd /d "%~dp0"

set "PYEXE="
for %%C in (py python python3) do if not defined PYEXE where %%C >nul 2>nul && set "PYEXE=%%C"
if not defined PYEXE goto NOPY

if /i "%PYEXE%"=="py" (set "RUN=py -3") else (set "RUN=%PYEXE%")

echo [启动] 使用 %RUN%
%RUN% -m carapp.main
if errorlevel 1 goto FAILED
exit /b 0

:FAILED
echo.
echo [提示] 程序异常退出。
echo        如果上面出现 ModuleNotFoundError: tkinter，说明这个 Python 精简掉了 tkinter，
echo        先运行  py -0  看一下本机有哪些版本，再用指定版本启动，例如：
echo            py -3.11 -m carapp.main
echo.
pause
exit /b 1

:NOPY
echo.
echo  [X] 没有找到 Python 解释器
echo.
echo  请先安装 Python 3.8 或更高版本（安装时务必勾选 "Add Python to PATH"）：
echo      https://www.python.org/downloads/
echo.
echo  安装完成后重新双击本文件即可，无需安装任何第三方库。
echo.
pause
exit /b 1
