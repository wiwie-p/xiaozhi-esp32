# 分支介绍

- develop：开发分支
- firmware：固件分支

现阶段，最新内容在develop分支内进行更新，待测试稳定后将合并到firmware分支内   
**故测试时请拉取develop分支内代码**   

***

# 编译说明
**一定要先配置项目再编译**   
**默认已安装esp-idf5.5.2及以上的环境**  
## 下载源码
有两种方式可以下载项目源码，分别为`git clone`命令与下载zip，建议通过`git clone`方式，建议使用`git clone`命令下载源码，因为同时会有分支信息
### git clone命令下载项目源码   
在文件资源管理器中导航到你想要存放项目的目录下，在空白处右键，在右键菜单中选择`在终端中打开`，并在终端中显示的路径为你想要存放项目的路径    
例如我选择存放项目的目录为为 `D:\projects\`，则我需要在终端中显示
> D:\projects>  

确保终端中显示的路径是想要放这个项目的目录    
导航到指定目录后，执行命令
> git clone git@gitee.com:max131/PlushMind.git

命令执行完后当前目录下便会出现`PlushMind`目录，至此，源码便下载完成    
**注意**:克隆后需要签出到`develop`分支，具体指令为
> git checkout develop

### 下载zip方式下载项目源码   
这种方式较为简单，打开浏览器导航到网址<https://gitee.com/max131/PlushMind>，并选择分支为**develop**，点击右侧**克隆/下载**按钮，在弹出界面中选择**下载zip**下载完成后会得到zip压缩包，将压缩包解压到指定的目录下，便可以得到当前的项目的develop分支源码

## 配置项目
1. 进入esp-idf环境，并导航到项目的根目录. 如我的项目目录为`D:\projects\PlushMind`，则我需要在激活esp-idf环境的终端中执行cd命令，导航到项目的根目录   
eg:
    > cd D:\projects\PlushMind
2. 在ESP-IDF环境下进入PlushMind文件夹后，执行`idf.py set-target esp32s3`命令，设置项目目标为esp32s3。  
3. 执行`idf.py menuconfig`命令，进入菜单配置界面，进行项目配置。
    1. 在项目根目录下执行`idf.py menuconfig`命令，进入菜单配置界面.    
    2. 通过方向键使光标移至Xiaozhi Assistant选项，并按回车键
    3. 选Board Type选项进去选择board类型为`Plushmind 第一版(面包板)`
    4. 按s键保存配置后按esp键退出菜单配置界面。
至此，项目配置完成

## 编译项目
在项目根目录下执行`idf.py build`命令，编译项目     

## 烧写项目
### 终端烧写
打开激活esp-idf环境的终端.导航到项目根目录，执行`idf.py -p (串口) flash`命令
eg:
> idf.py -p COM3 flash

### VSCode烧写
打开VSCode，打开项目根目录，点击vscode下面的falsh按钮

## 串口监视
### 终端串口监视
打开激活esp-idf环境的终端，执行`idf.py -p (串口) monitor`命令   
eg:
> idf.py -p COM3 monitor

### VSCode串口监视
打开VSCode，打开项目根目录，点击vscode下面的monitor按钮

***
建议使用终端进行操作，vscode可能有一些未知问题，当处于终端环境时，也可使用`idf，py -p (串口) build flash monitor`命令完成编译、烧写、监视操作，较为方便   
