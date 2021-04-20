TASKKILL /f /im putty.exe
TIMEOUT /t 3 /nobreak
START "" "E:\Program Files (x86)\PuTTY\putty.exe" -serial COM6 -sercfg 1200,8,n,1,N
TIMEOUT /t 5 /nobreak
TASKKILL /f /im putty.exe
TIMEOUT /t 1 /nobreak
copy "P:\Documents\RasPi Pico\i2c slave testing\build\i2c_slave_test.uf2" i:
TIMEOUT /t 3 /nobreak
START "" "E:\Program Files (x86)\PuTTY\putty.exe" -serial COM6 -sercfg 115200,8,n,1,N
EXIT