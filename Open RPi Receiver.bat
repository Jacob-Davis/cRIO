cd C:\Users\Driver\camera
netcat-win32-1.12\nc.exe -L -p 1180 | "MPlayer-x86_64-r36573+ge11983b\mplayer.exe" -vo direct3d -fps 14 -benchmark -nosound -cache 1024 -
timeout 10000