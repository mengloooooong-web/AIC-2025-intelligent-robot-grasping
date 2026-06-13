此脚本为安装 inuSDK 简化脚本

1. 修改为 unix 文件格式
   sed -i 's/\r//g' *.sh
2. 把 inudev*.deb 格式 或者Inu*.tar.gz 格式安装包放到 inuInstall.sh 同一目录下
3. 执行  inuInstall.sh, 按照提示一步一步来
   注意： 只支持 single-module 或者  multi-moudle
4. start, stop service
   sudo systemctl start inuservice.service              (单模组)
   sudo systemctl stop inuservice.service              （单模组）
   sudo systemctl start inuservice@{1..2}.service      （多模组）
   sudo systemctl stop inuservice@{1..2}.service       （多模组）
   sudo systemctl start inuservice@1.service           （多模组）
   sudo systemctl stop inuservice@1.service            （多模组）


English:
   0. plugin 2 sensors to USB port
   1. sed -i  's/\r//g'  *.sh
   2. put inudev_4.*.deb  or  Inu*.tar.gz  in this directory
   3. sudo ./inuInstall.sh
   4. select 2: install multi-module SDK!
   5. when input camera number: put 2
   6. select bind usb port: 1
   7. start, stop service
      sudo systemctl start inuservice@{1..2}.service
      sudo systemctl stop inuservice@{1..2}.service
      sudo systemctl start inuservice@1.service
      sudo systemctl stop inuservice@1.service

      then service is ok.