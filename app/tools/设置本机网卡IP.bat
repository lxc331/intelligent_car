@echo off
setlocal
title 设置本机网卡 IP（连小车用）

net session >nul 2>&1
if errorlevel 1 goto NOADMIN

echo.
echo  本工具把"电脑的以太网口"设成和小车同一网段，好让程序能找到小车。
echo.
echo  默认参数：IP 192.168.150.100 / 掩码 255.255.255.0 / 不设网关
echo  （小车自己是 192.168.150.3，两者必须在同一网段）
echo.
echo  ---------------------------------------------------------------
echo  先看看本机有哪些网卡（找到接网线那个，比如"以太网 3"）：
echo  ---------------------------------------------------------------
netsh interface show interface
echo.
set /p NIC=请输入网卡名称（直接回车用 以太网 3）:
if "%NIC%"=="" set NIC=以太网 3
set /p IPADDR=请输入要设置的 IP（直接回车用 192.168.150.100）:
if "%IPADDR%"=="" set IPADDR=192.168.150.100

echo.
echo  即将执行：
echo    netsh interface ip set address name="%NIC%" static %IPADDR% 255.255.255.0
echo.
pause

netsh interface ip set address name="%NIC%" static %IPADDR% 255.255.255.0
if errorlevel 1 goto FAILED

echo.
echo  [OK] 已设置 %NIC% = %IPADDR%
echo       用 ping 192.168.150.3 测一下，通了就双击 app 目录下的 run.bat。
echo.
pause
exit /b 0

:FAILED
echo.
echo  [X] 设置失败。常见原因：网卡名称写错了（区分空格），或者没以管理员身份运行。
echo.
pause
exit /b 1

:NOADMIN
echo.
echo  [X] 当前不是管理员权限，netsh 改不了 IP。
echo.
echo  请关掉本窗口，右键本文件 - 以管理员身份运行。
echo.
pause
exit /b 1
