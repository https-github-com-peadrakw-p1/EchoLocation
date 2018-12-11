# Installing OpenCV

This tutorial shows how to install opencv in ubuntu 18.04

First make sure your computer is up date

```
$ sudo apt-get update
$ sudo apt-get upgrade
```

Secondly install the developer tools

```
$ sudo apt-get install build-essential cmake unzip pkg-config
```

Next make sure you are able to load basic image formats

```
$ sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
```

Next lets include video io packages

```
$ sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
$ sudo apt-get install libxvidcore-dev libx264-dev
$ sudo apt-get install libv4l-dev
```

OpenCV’s highgui module relies on the GTK library for GUI operations. INstall that next

```
$ sudo apt-get install libgtk-3-dev
```

Install optomizing libariies

```
$ sudo apt-get install libatlas-base-dev gfortran
```

Finall install the python 3 headers

```
$ sudo apt-get install python3-dev
```

Download the official opencv library

```
$ cd ~
$ wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.4.zip
$ wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.4.zip
```

Unzip these modules

```
$ unzip opencv.zip
$ unzip opencv_contrib.zip
```

We are going to need to install some python packages the easiest way to do this is using the pip package manager. Check you have pip installed using

```
$ sudo apt install python-pip
```

We need to install opencl headers used by opencv

```
$ sudo apt-get install opencl-
$ sudo apt-get install ocl-icd-libopencl1
```

Make sure you have numpy installed

```
$ pip install --user numpy
```

Set up your opencv build. Always check that you dont have any errors after this step

```
$ cd ~/opencv-3.4.4/
$ mkdir build
$ cd build
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D INSTALL_C_EXAMPLES=OFF \
	-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.4.4/modules \
	-D PYTHON_EXECUTABLE=~/.virtualenvs/cv/bin/python \
	-D BUILD_EXAMPLES=ON ..
```

Compile your open cv set the j flag to the number of cores you have.

```
$ make -j4
```

Now that you have compiled opencv you can install it

```
$ sudo make install
$ sudo ldconfig
```

Verify that everything worked correctly

```
$ pkg-config --modversion opencv
```

Check that you have installed everything correctly:

```
$ python
>>> import cv2
>>> cv2.__version__
'3.4.1'
>>> quit()
```

## Authors

* **Carl Hildebrandt** - *Initial work* - [hildebrandt-carl](https://github.com/hildebrandt-carl)