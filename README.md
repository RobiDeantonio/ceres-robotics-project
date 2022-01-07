# CERES robotics Project

Agricultural robotics research project

## Comenzando 🚀

_These instructions allow you to get a copy of the project running on your local machine for development and testing purposes._

See **Deployment** to know how to deploy the project.


## About the Project
The Agrobot CERES is a mobile robotic platform created by the National University of Colombia and the Nueva Granada Military University with the aim of being able to measure, determine and autonomously carry out corrective actions on agricultural crops.
        
![](https://github.com/RobiDeantonio/ceres-robotics-project/blob/main/img/ceres.png)

### Pre-requisitos 📋

_What things do you need to install the software and how to install them._

**Supported:** Ubuntu 20.04 & Debian Buster (10)  
**Experimental:** Windows 10, Arch Linux  
`ROS Noetic` : <http://wiki.ros.org/noetic/Installation>  
`Catkin_make ROS Enviroment` : <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>  
`Arduino` : <https://www.arduino.cc/en/software>  
python => 3.6  

### Instalation 🔧

To install the repository, follow the next steps.

Locate your directory at:
```
$ cd /home/<user>/catkin_ws/src
```
and install the repository with the following commands

```
$ git init
$ git remote add origin https://github.com/RobiDeantonio/ceres-robotics-project.git

$ git branch              ## Show actual branches
$ git fetch --all
$ git branch -r | grep -v '\->' | while read remote; do git branch --track "${remote#origin/}" "$remote"; done
$ git pull --all

$ git checkout main       ## to work with main branch
```

## Running testing with ROS Enviroment  _ceres Branch in git_ ⚙️

Locate your directory and run following commands:
```
$ cd /home/<user>/catkin_ws/src
$ git checkout ceres      ## to work with ceres branch
$ cd /home/<user>/catkin_ws
$ catkin_make
$ roscore
```

In other Terminal:
Locate your directory and run following commands:
```
$ cd /home/<user>/catkin_ws
$ source devel/setup.bash 
$ rosrun ceres ceresGUI.py
```

## Construido con 🛠️


* [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu) - The Robotics framework
* [QT5 Designer](https://doc.qt.io/qt-5/qtdesigner-manual.html) - GUI
* [Interface GUI and Python](https://pypi.org/project/PyQt5/) - GUI and logic connections
* [Leaflet](https://leafletjs.com/) - Maps and geolocation


## Contributing 🖇️

Por favor lee el  para detalles de nuestro código de conducta, y el proceso para enviarnos pull requests.
Please read [CONTRIBUTING.md](https://github.com/RobiDeantonio/ceres-robotics-project/blob/main/CONTRIBUTING.md) for details of our code of conduct, and the process for submitting pull requests.

## GIDAM 📖

You can know more about the research group [GIDAM](https://www.umng.edu.co/sedes/bogota/facultad-de-ingenieria/centro-de-investigacion/gidam)


## Autores ✒️


* **Leonardo Enrique Solaque** - *Researcher* - [Leonardo](https://github)
* **Alexandra Velasco Vivas** - *Researcher* - [Alexandra](https://www.linkedin.com/in/alexandra-velasco-vivas-2446a62a/)
* **Adriana Riveros Guevara** - *Researcher* - [Alexandra](https://w)

* **Adrien Legrand** - *Contribution with thesis* - [Adrien](https://www.linkedin.com/in/adrienlegrand/)
* **Guillermo Sanchez Herrera** - *research assistant* - [Guillermo](https://www.linkedin.com/in/datacloudgui/)
 

## License 📄

This project is under the License - see the file [LICENSE.md](https://github.com/RobiDeantonio/ceres-robotics-project/blob/main/LICENSE) for details

## Expresiones de Gratitud 🎁

* Comenta a otros sobre este proyecto 📢
* Invita una cerveza 🍺 o un café ☕ a alguien del equipo. 
* Da las gracias públicamente 🤓.


