# Steps to compile and generate FMUs

### Create a `build` folder in `custom_fmus`.

### Start the ROS command 

### Use UTF-8 encoding 
If your name contains any special characters, you might not be able to build the project. Change the console encoding to `utf-8` by running 
```
chcp 65001
```
and specify the Python I/O encoding to `utf-8` 
```
set PYTHONIOENCODING=utf-8
```

### Install c++ packages using conan
From the ROS terminal enter the `build` folder and run 
```
conan config set general.revisions_enabled=True
conan remote add zeabuz https://zeabuz.jfrog.io/artifactory/api/conan/default-conan
conan user -p <password> -r zeabuz <username>
conan install .. -s build_type=Debug -s compiler.libcxx=libstdc++11 --build=missing
```

### Generate the project files
From the `build` folder run 
```
cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
```

### Build the FMUS
From the `build` folder run 
```
cmake --build . --config Debug
```
