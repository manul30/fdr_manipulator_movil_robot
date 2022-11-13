# fdr_manipulator_movil_robot
Para la correcta instalación del paquete, ejecutar los siguientes comandos:

```
# Ubicarse en el workspace

$ roscd

$ cd ./src

$ git clone https://github.com/manul30/fdr_manipulator_movil_robot.git

$ cd ..

$ catkin_make

```

Para visualizar el modelo de Kuka KR4 R600 Agilus
```
# En Rviz

$ roslaunch kuka_kr4_description display.launch
 
# En Gazebo

$ roslaunch kuka_kr4_description gazebo.launch

```
![text image](https://s3.us-west-2.amazonaws.com/secure.notion-static.com/fb694991-9abd-410d-b4e4-cf90c6787939/Untitled.png?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Content-Sha256=UNSIGNED-PAYLOAD&X-Amz-Credential=AKIAT73L2G45EIPT3X45%2F20221113%2Fus-west-2%2Fs3%2Faws4_request&X-Amz-Date=20221113T032357Z&X-Amz-Expires=86400&X-Amz-Signature=7d59d25fc21ea8c101468556007fc09bfd65a3fc7a0d76f01b3e13a13a8965ff&X-Amz-SignedHeaders=host&response-content-disposition=filename%3D%22Untitled.png%22&x-id=GetObject)
