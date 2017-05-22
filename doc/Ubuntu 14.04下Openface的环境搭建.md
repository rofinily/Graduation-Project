# Ubuntu 14.04下Openface的环境搭建

系统：`Ubuntu 14.04` 64位桌面操作系统

参考：[OpenFace Setup Guide](http://cmusatyalab.github.io/openface/setup/)

### 1、Linux使用超级用户

```shell
su
```

### 2、安装前准备工作

安装必要的程序，可以用下面的`shell`脚本，也可以一个一个的进行安装。

```shell
#!/bin/sh
sudo apt-get install build-essential -y
sudo apt-get install cmake -y
sudo apt-get install curl -y
sudo apt-get install gfortran -y
sudo apt-get install git -y
sudo apt-get install libatlas-dev -y
sudo apt-get install libavcodec-dev -y
sudo apt-get install libavformat-dev -y
sudo apt-get install libboost-all-dev -y
sudo apt-get install libgtk2.0-dev -y
sudo apt-get install libjpeg-dev -y
sudo apt-get install liblapack-dev -y
sudo apt-get install libswscale-dev -y
sudo apt-get install pkg-config -y
sudo apt-get install python-dev -y
sudo apt-get install python-pip -y
sudo apt-get install wget -y
sudo apt-get install zip –y
```

### 3、安装必要的库

```shell
pip2 install numpy scipy pandas  
pip2 install scikit-learn scikit-image
```

注意：

- 如果出现某一个安装失败的情况，可以一个一个的安装

- 提高`pip`安装速度

  可以更换`pip`镜像加快下载速度
  建立`./pip/pip.conf`，输入以下内容（或者其他可用镜像）：

  ```cfg
  [global]  
  timeout = 6000  
  index-url = http://pypi.douban.com/simple  

  [install]  
  use-mirrors = true  
  mirrors = http://pypi.douban.com/
  ```


- 报错：`SSLError: The read operation timed out`

  可以用下列指令将延时加长，设置timeout

  ```shell
  pip2 -install scikit-image --timeout 100
  ```

### 4、安装Torch

- 安装依赖

  ```shell
  curl -s https://raw.githubusercontent.com/torch/ezinstall/master/install-deps  | bash -e
  ```


- 从远程代码库下载torch并安装

  ```shell
  git clone https://github.com/torch/distro.git ~/torch --recursive  
  cd ~/torch && ./install.sh
  ```

- `luarocks`安装依赖

  ```shell
  ~/torch/install/bin/luarocks install dpnn
  ~/torch/install/bin/luarocks install nn
  ~/torch/install/bin/luarocks install optim
  ~/torch/install/bin/luarocks install csvigo
  ~/torch/install/bin/luarocks install cunn
  ~/torch/install/bin/luarocks install fblualib
  ~/torch/install/bin/luarocks install torchx
  ```


- 验证是否安装依赖成功

  用`th`命令验证

注意：

- `git clone`更新网络老中断

  ```shell
  git submodule update --init –recursive
  ```

  或者`torch`目录下的`Update.sh`，建议用运行`Update.sh`解决

- 出现错误：

  ``` shell
  error: RPCfailed; result=56, HTTP code = 200| 0 bytes/s
  fatal: Theremote end hung up unexpectedly
  fatal: earlyEOF
  fatal:index-pack failed
  ```

### 5、安装`OpenCV`

`OpenCV`版本为2.4.11，下载地址：[https://github.com/Itseez/opencv/archive/2.4.11.zip](https://github.com/Itseez/opencv/archive/2.4.11.zip)

编译参考：[http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html](http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html)

- 指令下载：

  ```shell
  cd ~
  mkdir -p src
  cd src
  curl -L https://github.com/Itseez/opencv/archive/2.4.11.zip -o opencv.zip
  ```

- 解压

  ```shell
  unzip opencv.zip
  ```

- 编译

  ```shell
  cd opencv-2.4.11
  mkdir release
  cd release
  cmake -D CMAKE_BUILD_TYPE=RELEASE -d CMAKE_INSTALL_PREFIX=/usr/local ..
  make -j8
  make install
  ```

- 验证

  ```shell
  python
  #在python命令行中，如果import成功则代表编译成功
  >>> import cv2
  ```

### 6、安装`dlib`

安装`dlib` v18.16

下载地址：[https://github.com/davisking/dlib/releases/download/v18.16/dlib-18.16.tar.bz2](https://github.com/davisking/dlib/releases/download/v18.16/dlib-18.16.tar.bz2)

- 安装编译

  ```shell
  mkdir -p ~/src
  cd ~/src
  tar -xf dlib-18.16.tar.bz2
  mkdir build
  cd build
  cmake ../../tools/python
  cmake --build . --config release
  cp dilb.so /usr/local/lib/python2.7/dist-packages
  ```

- 确保

  在上一步的最后一条命令中，确保路径在默认的`Python`路径，可以在`Python`解释器里面用`sys.path`查找

- 验证

  ```shell
  python
  #在python命令行中，如果import成功则代表编译成功
  >>> import dlib
  ```


### 7、`Git`获取`Openface`

- 从远程库下载`Openface`

  ```shell
  git clone https://github.com/cmusatyalab/openface.git
  git submodule init
  git submodule update
  ```

- 在`Openface`根目录执行

  ```shell
  #需要确保dlib和opencv安装成功
  sudo python2 setup.py install
  ```

- 获取模型

  ```shell
  models/get-models.sh
  ```


### 8、运行demo

- 运行demo1：

  ```shell
  ./demos/compare.py images/examples/{lennon,clapton}
  ```


- 运行demo2：

  ```shell
  ./demos/classifier.py infermodels/openface/celeb-classifier.nn4.small2.v1.pkl ./images/examples/carell.jpg
  ```


- 运行demo3：

  ```shell
  ./demos/web/start-servers.sh
  ```