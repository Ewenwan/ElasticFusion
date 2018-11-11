# 稠密SLAM
     稠密SLAM重建目前也相对比较成熟，
     从最开始的KinectFusion（TSDF数据结构 + ICP），
     到后来的InfiniTAM（用哈希表来索引很稀疏的voxel）, 
     ElasticFusion（用surfel点表示模型并用非刚性的图结构），
     DynamicFusion（引入了体翘曲场这样深度数据通过体翘曲场的变换后，
         才能融入到TSDF数据结构中，完成有非刚性物体的动态场景重建）都做的比较成熟。
     工业界实现非常好的是微软的HoloLens，在台积电的24核DSP上把mesh simplification这些操作都搞了上去。


# 文章评价
     1、基于 RGB-D 稠密的三维重建，一般使用网格模型融合点云，
        ElasticFusion 是为数不多使用 surfel 模型表示的。
     2、传统的 SLAM 算法一般通过不断优化相机轨迹或者特征点的方式，来提高轨迹估计或者重建的精度，
        这篇文章采用不断优化重建的 map 的方式，提高重建和位姿估计的精度。
     3、优化采用 deformation graph 的形式，和 DynamicFusion 中优化的方式如出一辙。
     4、融合了重定位算法（当相机跟丢时，重新计算相机的位姿）。
     5、算法使用 OpenGL 对点云进行更新、融合、显示和投影。
     6、算法融合 RGB-D 图像进行位姿估计，对于 RGB 图像通过颜色一致性约束计算位姿，
        对于点云通过 ICP 算法计算位姿，ElasticFusion 融合了二者。
     7、适合重建房间大小的场景，代码没有做优化，重建较大场景时，代码不能适用。

# 算法流程
     1、ElasticFusion 通过 RGB-D 图像配准计算位姿，
        根据输入点云利用 ICP 算法配准计算位姿的方法参见博客：
        http://blog.csdn.net/fuxingyin/article/details/51425721
        
        icp点云配准误差       sum(Pi - (R*Pj+t))
        + 直接法 重投影光度误差  sum(Ik(pi) -  Ik-1(投影到2D(R*恢复3D(pj) + t)) )
        
     2、计算相机位姿如果误差大于设定阈值，表示跟踪失败，启动重定位算法；
        如果误差小于设定阈值，则进入下一部分。
     3、利用 Random Ferns (图像编码) 进行全局的回环检测算法
        检测是否存在全局的回环，如果存在全局的回环，假设当前帧为 Icur
        检测到和第 i 帧存在回环；再利用第一部中的跟踪算法计算当前帧和第 i
        帧之间的位姿，计算得到位姿变换后，在图像中均匀抽取一些点，建立约束，
        优化 node 参数
        
        输入图像 -> 计算 图像块 的码值 -> 根据码值计算和list表中帧之间的相似度
        -> 根据相似度判别是否加入关键帧或者做重定位。
        
     4、在第 3 部中如果不存在全局的回环，则检测是否存在局部的回环，
        如果存在局部的回环，则同第 3 步，进行位姿估计，并且建立约束，优化 node 参数。
     5、计算得到相机位姿后，将当前帧的点云和重建好的做融合，融合使用 openGL 的 shading language，
        如果在存在局部的或者全局的回环，在使用 openGL 进行点的融合时候，将优化之后的节点变量，作用于全部的点。
     6、融合到全局模型中后再用 openGL 投影得到当前视角下可以看到的点，用来对下一帧图像配准。
     
http://www.cnblogs.com/zonghaochen/p/8643316.html     
     
     
     
# surfel 表示模型
     实时的 RGB-D 重建一般用 KinectFusion 算法中提到的 TSDF 模型，这篇文章用 surfel 模型（点表示模型），
          对于每个点，存储了:
          点的位置信息：(x,y,z)
          面片得半径: r
          法向量： n
          颜色信息：(R,G,B)
          点的获取时间信息：t

          在进行点的融合更新时：点的位置信息，法向量，和颜色信息的
          更新方法类似于 KinectFusion 采用加权融合的方式，
          面片的半径通过场景表面离相机光心的距离的求得，
          距离越大，面片的半径越大。
          
#  Randomized Ferns 在 ElasticFusion 中地位和作用
     
     ElasticFusion 通过 Randomized Ferns 重定位和回环检测，Randomized ferns 是实时的重定位和回环检测算法。
     
     Randomized Ferns 对关键帧图像编码，采用特殊的编码存储方式，加快图像比较的效率。
     
     https://blog.csdn.net/fuxingyin/article/details/51436430
     



# 安装
     2. 编译前准备
     2.1 硬件配置

     本文使用的硬件配置如下：
     + Intel i7-7700HQ
     + NVIDIA® GeForce® GTX 1060（GPU必不可少，
        否则无法完成编译,可参见ElasticFusion页面8.FAQ内容）
     + 16GB DDR4 2400MHz 内存
     2.2 软件配置
         Ubuntu16.04系统，ElasticFusion也支持Ubuntu14.04和Unbuntu15.04，
     2.3 库依赖
         CMake　——　众所周知的跨平台的安装（编译）工具，推荐安装gui版本，安装命令如下:
         sudo apt-get install cmake-qt-gui
         git —— 版本控制软件，用于下载部分需要源码编译的依赖库，安装命令如下:
         sudo apt-get install git
         build-essential　——　Linux下的C/C++编译环境及依赖，如gcc和g++，安装命令如下:
         sudo apt-get install build-essential
         libusb-1.0-0-dev　——　用户空间的USB驱动库，安装命令如下:
         sudo apt-get install libusb-1.0-0-dev
         libudev-dev　——　用户空间的设备文件管理库，安装命令如下:
         sudo apt-get install libudev-dev
         openjdk-7-jdk ——　开源的Java开发环境（因为我已经装了Oracle的JDK，所以这里没有装），安装命令如下:
         sudo apt-get install openjdk-7-jdk
         freeglut3-dev —— 开源的OpenGL库，安装命令如下:
         sudo apt-get install freeglut3-dev
         python-vtk —— python版的图形可视化库（vtk），安装命令如下:
         sudo apt-get install python-vtk
         libvtk-java —— jave版的图形可视化库（vtk），安装命令如下:
         sudo apt-get install libvtk-java
         libglew-dev —— OpenGL库，安装命令如下:
         sudo apt-get install libglew-dev
         libsuitesparse-dev —— 稀疏矩阵运算库，安装命令如下:
         sudo apt-get install libsuitesparse-dev
         libeigen3-dev —— 著名的矩阵运算库，安装命令如下:
         sudo apt-get install libeigen3-dev
         zlib1g-dev —— 数据压缩库，安装命令如下:
         sudo apt-get install zlib1g-dev
         libjpeg-dev —— JPEG图像压缩库，安装命令如下:
         sudo apt-get install libjpeg-dev
         上述的库依赖均可以通过Ubuntu自带的apt-get安装，之所以分别列开，
         也是为了让读者明白各个依赖库的作用和功能，一键安装命令如下:
         sudo apt-get install -y cmake-qt-gui git build-essential 
         libusb-1.0-0-dev libudev-dev openjdk-7-jdk freeglut3-dev 
         libglew-dev libsuitesparse-dev libeigen3-dev zlib1g-dev libjpeg-dev
         
         CUDA版本大于7.0。 
         OpenNI2 
               git clone https://github.com/occipital/OpenNI2
               cd OpenNI2
               mkdir build && cd build
               cmake ..
               make -j8
               sudo make install
               sudo ldconfig 
         Pangolin
               Pangolin是对OpenGL进行封装的轻量级的OpenGL输入/输出和视频显示的库。
               可以用于3D视觉和3D导航的视觉图，可以输入各种类型的视频、
               并且可以保留视频和输入数据用于debug。
               编译命令如下:
          
               git clone https://github.com/stevenlovegrove/Pangolin.git
               cd Pangolin
               mkdir build && cd build
               cmake ..
               make -j8
               sudo make install
               sudo ldconfig 
     3. 编译过程

     首先通过git下载源码，命令如下:
     git clone https://github.com/mp3guy/ElasticFusion.git
     ElasticFusion文件夹中包含三个子文件夹，分别是Core，GPUTest和GUI，编译顺序如下所示。
     3.1 Core编译

          cd ElasticFusion
          cd Core
          mkdir build && cd build
          cmake ..
          make -j8
          sudo make install
          sudo ldconfig

     3.2 GPUTest

          cd ElasticFusion
          cd GPUTest
          mkdir build && cd build
          cmake ..
          make -j8
          sudo make install
          sudo ldconfig

     3.3 GUI

          cd ElasticFusion
          cd GUI
          mkdir build && cd build
          cmake ..
          make -j8
          sudo make install
          sudo ldconfig
     
     4. 运行datasets

          在此处下载数据集, 运行命令如下:
          www.doc.ic.ac.uk/~sleutene/datasets/elasticfusion/dyson_lab.klg   1.1 GB
          
          cd ElasticFusion
          cd GUI/build
          ./ElasticFusion -l dyson_lab.klg


 
# ElasticFusion #

Real-time dense visual SLAM system capable of capturing comprehensive dense globally consistent surfel-based maps of room scale environments explored using an RGB-D camera.

# Related Publications #
Please cite this work if you make use of our system in any of your own endeavors:

* **[ElasticFusion: Real-Time Dense SLAM and Light Source Estimation](http://www.thomaswhelan.ie/Whelan16ijrr.pdf)**, *T. Whelan, R. F. Salas-Moreno, B. Glocker, A. J. Davison and S. Leutenegger*, IJRR '16
* **[ElasticFusion: Dense SLAM Without A Pose Graph](http://thomaswhelan.ie/Whelan15rss.pdf)**, *T. Whelan, S. Leutenegger, R. F. Salas-Moreno, B. Glocker and A. J. Davison*, RSS '15

# 1. What do I need to build it? #

## 1.1. Ubuntu ##

* Ubuntu 14.04, 15.04 or 16.04 (Though many other linux distros will work fine)
* CMake
* OpenGL
* [CUDA >= 7.0](https://developer.nvidia.com/cuda-downloads)
* [OpenNI2](https://github.com/occipital/OpenNI2)
* SuiteSparse
* Eigen
* zlib
* libjpeg
* [Pangolin](https://github.com/stevenlovegrove/Pangolin)
* [librealsense] (https://github.com/IntelRealSense/librealsense) - Optional (for Intel RealSense cameras)

Firstly, add [nVidia's official CUDA repository](https://developer.nvidia.com/cuda-downloads) to your apt sources, then run the following command to pull in most dependencies from the official repos:

```bash
sudo apt-get install -y cmake-qt-gui git build-essential libusb-1.0-0-dev libudev-dev openjdk-7-jdk freeglut3-dev libglew-dev cuda-7-5 libsuitesparse-dev libeigen3-dev zlib1g-dev libjpeg-dev
```

Afterwards install [OpenNI2](https://github.com/occipital/OpenNI2) and [Pangolin](https://github.com/stevenlovegrove/Pangolin) from source. Note, you may need to manually tell CMake where OpenNI2 is since Occipital's fork does not have an install option. It is important to build Pangolin last so that it can find some of the libraries it has optional dependencies on. 

When you have all of the dependencies installed, build the Core followed by the GUI. 

## 1.2. Windows - Visual Studio ##
* Windows 7/10 with Visual Studio 2013 Update 5 (Though other configurations may work)
* [CMake] (https://cmake.org/)
* OpenGL
* [CUDA >= 7.0](https://developer.nvidia.com/cuda-downloads)
* [OpenNI2](https://github.com/occipital/OpenNI2)
* [SuiteSparse] (https://github.com/jlblancoc/suitesparse-metis-for-windows)
* [Eigen] (http://eigen.tuxfamily.org)
* [Pangolin](https://github.com/stevenlovegrove/Pangolin)
  * zlib (Pangolin can automatically download and build this)
  * libjpeg (Pangolin can automatically download and build this)
* [librealsense] (https://github.com/IntelRealSense/librealsense) - Optional (for Intel RealSense cameras)

Firstly install cmake and cuda. Then download and build from source OpenNI2, SuiteSparse. Next download Eigen (no need to build it since it is a header-only library). Then download and build from source Pangolin but pay attention to the following cmake settings. There will be a lot of dependencies where path was not found. That is OK except OPENNI2 and EIGEN3 (those should be set to valid paths). You also need to set MSVC_USE_STATIC_CRT to false in order to correctly link to ElasticFusion projects. Also, you can set BUILD_EXAMPLES to false since we don't need them and some were crashing on my machine.

Finally, build Core and GUI.


# 2. Is there an easier way to build it? #
Yes, if you run the *build.sh* script on a fresh clean install of Ubuntu 14.04, 15.04, or 16.04, enter your password for sudo a few times and wait a few minutes all dependencies will get downloaded and installed and it should build everything correctly. This has not been tested on anything but fresh installs, so I would advise using it with caution if you already have some of the dependencies installed.

# 3. Installation issues #

***`#include <Eigen/Core>` not found***

```bash
sudo ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
sudo ln -sf /usr/include/eigen3/unsupported /usr/include/unsupported
```

***invalid use of incomplete type ‘const struct Eigen ...***

Pangolin must be installed AFTER all the other libraries to make use of optional dependencies.

***GLSL 3.30 is not supported. Supported versions are 1.10, 1.20, 1.30, 1.00 ES and 3.00 ES***

Make sure you are running ElasticFusion on your nVidia GPU. In particular, if you have an Optimus GPU
- If you use Prime, follow instructions [here](http://askubuntu.com/questions/661922/how-am-i-supposed-to-use-nvidia-prime)
- If you use Bumblebee, remember to run as `optirun ./ElasticFusion`

# 4. How do I use it? #
There are three subprojects in the repo:

* The *Core* is the main engine which builds into a shared library that you can link into other projects and treat like an API. 
* The *GUI* is the graphical interface used to run the system on either live sensor data or a logged data file. 
* The *GPUTest* is a small benchmarking program you can use to tune the CUDA kernel launch parameters used in the main engine. 

The GUI (*ElasticFusion*) can take a bunch of parameters when launching it from the command line. They are as follows:

* *-cal <calibration>* : Loads a camera calibration file specified as *fx fy cx cy*.
* *-l <logfile>* : Processes the specified .klg log file.
* *-p <poses>* : Loads ground truth poses to use instead of estimated pose.
* *-c <confidence>* : Surfel confidence threshold (default *10*).
* *-d <depth>* : Cutoff distance for depth processing (default *3*m).
* *-i <icp>* : Relative ICP/RGB tracking weight (default *10*).
* *-ie <error>* : Local loop closure residual threshold (default *5e-05*).
* *-ic <count>* : Local loop closure inlier threshold (default *35000*).
* *-cv <covariance>* : Local loop closure covariance threshold (default *1e-05*).
* *-pt <photometric>* : Global loop closure photometric threshold (default *115*).
* *-ft <threshold>* : Fern encoding threshold (default *0.3095*).
* *-t <time>* : Time window length (default *200*).
* *-s <skip>* : Frames to skip at start of log.
* *-e <end>* : Cut off frame of log.
* *-f* : Flip RGB/BGR.
* *-icl* : Enable this if using the [ICL-NUIM](http://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html) dataset (flips normals to account for negative focal length on that data).
* *-o* : Open loop mode.
* *-rl* : Enable relocalisation.
* *-fs* : Frame skip if processing a log to simulate real-time.
* *-q* : Quit when finished a log.
* *-fo* : Fast odometry (single level pyramid).
* *-nso* : Disables SO(3) pre-alignment in tracking.
* *-r* : Rewind and loop log forever. 
* *-ftf* : Do frame-to-frame RGB tracking. 
* *-sc* : Showcase mode (minimal GUI).

Essentially by default *./ElasticFusion* will try run off an attached ASUS sensor live. You can provide a .klg log file instead with the -l parameter. You can capture .klg format logs using either [Logger1](https://github.com/mp3guy/Logger1) or [Logger2](https://github.com/mp3guy/Logger2). 

# 5. How do I just use the Core API? #
The libefusion.so shared library which gets built by the Core is what you want to link against.

An example of this can be seen in the GUI code. Essentially all you need to do is utilise the provided Findefusion.cmake file in GUI/src and include the following in your CMakeLists.txt file:

    find_package(efusion REQUIRED)
    include_directories(${EFUSION_INCLUDE_DIR})
    target_link_libraries(MyProject ${EFUSION_LIBRARY})
    
To then use the Core API, make sure to include the header file in your source file:
```cpp
    #include <ElasticFusion.h>
```

Initialise the static configuration parameters once somewhere at the start of your program (this [smells](http://en.wikipedia.org/wiki/Code_smell), but whatever):
```cpp
    Resolution::getInstance(640, 480);
    Intrinsics::getInstance(528, 528, 320, 240);
```

Create an OpenGL context before creating an ElasticFusion object, as ElasticFusion uses OpenGL internally. You can do this whatever way you wish, using Pangolin is probably easiest given it's a dependency:
```cpp
    pangolin::Params windowParams;
    windowParams.Set("SAMPLE_BUFFERS", 0);
    windowParams.Set("SAMPLES", 0);
    pangolin::CreateWindowAndBind("Main", 1280, 800, windowParams);
```

Make an ElasticFusion object and start using it:
```cpp
    ElasticFusion eFusion;
    eFusion.processFrame(rgb, depth, timestamp, currentPose, weightMultiplier);
```

See the source code of MainController.cpp in the GUI source to see more usage.

# 6. Datasets #

We have provided a sample dataset which you can run easily with ElasticFusion for download [here](http://www.doc.ic.ac.uk/~sleutene/datasets/elasticfusion/dyson_lab.klg). Launch it as follows:

```bash
./ElasticFusion -l dyson_lab.klg
```

# 7. License #
ElasticFusion is freely available for non-commercial use only.  Full terms and conditions which govern its use are detailed [here](http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/) and in the LICENSE.txt file.

# 8. FAQ #
***What are the hardware requirements?***

A [very fast nVidia GPU (3.5TFLOPS+)](https://en.wikipedia.org/wiki/List_of_Nvidia_graphics_processing_units#GeForce_900_Series), and a fast CPU (something like an i7). If you want to use a non-nVidia GPU you can rewrite the tracking code or substitute it with something else, as the rest of the pipeline is actually written in the OpenGL Shading Language. 

***How can I get performance statistics?***

Download [Stopwatch](https://github.com/mp3guy/Stopwatch) and run *StopwatchViewer* at the same time as ElasticFusion. 

***I ran a large dataset and got assert(graph.size() / 16 < MAX_NODES) failed***

Currently there's a limit on the number of nodes in the deformation graph down to lazy coding (using a really wide texture instead of a proper 2D one). So we're bound by the maximum dimension of a texture, which is 16384 on modern cards/OpenGL. Either fix the code so this isn't a problem any more, or increase the modulo factor in *Shaders/sample.geom*. 

***I have a nice new laptop with a good GPU but it's still slow***

If your laptop is running on battery power the GPU will throttle down to save power, so that's unlikely to work (as an aside, [Kintinuous](https://github.com/mp3guy/Kintinuous) will run at 30Hz on a modern laptop on battery power these days). You can try disabling SO(3) pre-alignment, enabling fast odometry, only using either ICP or RGB tracking and not both, running in open loop mode or disabling the tracking pyramid. All of these will cost you accuracy. 

***I saved a map, how can I view it?***

Download [Meshlab](http://meshlab.sourceforge.net/). Select Render->Shaders->Splatting. 

***The map keeps getting corrupted - tracking is failing - loop closures are incorrect/not working***

Firstly, if you're running live and not processing a log file, ensure you're hitting 30Hz, this is important. Secondly, you cannot move the sensor extremely fast because this violates the assumption behind projective data association. In addition to this, you're probably using a primesense, which means you're suffering from motion blur, unsynchronised cameras and rolling shutter. All of these are aggravated by fast motion and hinder tracking performance. 

If you're not getting loop closures and expecting some, pay attention to the inlier and residual graphs in the bottom right, these are an indicator of how close you are to a local loop closure. For global loop closures, you're depending on [fern keyframe encoding](http://www.doc.ic.ac.uk/~bglocker/pdfs/glocker2015tvcg.pdf) to save you, which like all appearance-based place recognition methods, has its limitations. 

***Is there a ROS bridge/node?***

No. The system relies on an extremely fast and tight coupling between the mapping and tracking on the GPU, which I don't believe ROS supports natively in terms of message passing. 

***This doesn't seem to work like it did in the videos/papers***

A substantial amount of refactoring was carried out in order to open source this system, including rewriting a lot of functionality to avoid certain licenses and reduce dependencies. Although great care was taken during this process, it is possible that performance regressions were introduced and have not yet been discovered.
