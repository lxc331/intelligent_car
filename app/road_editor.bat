@echo off
setlocal
title 沙盘路网标注 · ClawPi
cd /d "%~dp0"

rem ===== 独立的沙盘路网标注窗口 =====
rem 双击即可；也可以在命令行传工程路径：
rem   road_editor.bat "app\savedMap\sandbox_nav_ready_20261101\sandbox.json"

set "PYEXE="
for %%C in (py python python3) do if not defined PYEXE where %%C >nul 2>nul && set "PYEXE=%%C"
if not defined PYEXE goto NOPY

if /i "%PYEXE%"=="py" (set "RUN=py -3") else (set "RUN=%PYEXE%")

echo [启动] 使用 %RUN%
%RUN% -m carapp.road_editor %*
if errorlevel 1 goto FAILED
exit /b 0

:FAILED
echo.
echo [提示] 标注器异常退出，看看上面的报错。
pause
exit /b 1

:NOPY
echo.
echo  [X] 没有找到 Python 解释器
echo      请先安装 Python 3.8+，安装时勾选 "Add Python to PATH"。
pause
exit /b 1
