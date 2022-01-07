# CERES robotics Project

Agricultural robotics research project

## Comenzando 🚀

_These instructions allow you to get a copy of the project running on your local machine for development and testing purposes._

See **Deployment** to know how to deploy the project.


## About the Project
The Agrobot CERES is a mobile robotic platform created by the National University of Colombia and the Nueva Granada Military University with the aim of being able to measure, determine and autonomously carry out corrective actions on agricultural crops.
        
![](https://lh3.googleusercontent.com/SQSq68_wluL-Sua5yJdNOplXyPwcxMIDKClqbrJWFOfbxiXFQGqnV6CP-WB4hHHm0ZABOuXlNzIiNsdBz30kqaPf2Gcjt06Q4fNjNz4BicTCJrrsyK_hLh3vh4ohRK7y0raRBVn-5v2QToFJl3N1ToqlGuSzsn8ivBS0x5hxMoUP8Uojtpogw1GehAPDYkchalunXz2VscMhDZX9kty-DVStgTQD9bUDBJsgBv62_JClFF8miWbWANVyonTPbrXuVtB2IoVUNbxz_qUrn6Xyr7tf_kc00XHuzS7kKxNEL1J0q3UBrh6qpuU46vFY-SrNwdzeM93DMrMkjXIAg6Xq_F89pVE0mSTIpf6ES_e8VdKbwJDJ6trLmm_pulZxINoz6RIk6w_MczPw2Qcm1dBqTL3fZaK85PGIUMk3-54dAljCnOJGSN9nW_mIdCNbPJrwF_q7yE-w-Itl-AEYdNOnT2-xPA300xpZHYBds6K6xmGPtMsqxOmq60waTsgmAPwM70vKVNq7ugKAk2Tfqq08V5XReT8i39Wkl6rCFw2ixaHO9JH0LwRs14sTp2RahUtoIe-mkn93dvVaN-unM4c3ecLE5r8PzYwAw4FH2OLlhWGGaZbzX27GjdgPsK4VF00zjCSf4AUqdRf47AIxArBZrQ9yuSj87SSTYzjzxsx60bZZsuXE25JLmyr8B_ayIHu-qxBuF1EdHpwoBnxPensvNbG6=w1068-h670-no?authuser=1)

### Pre-requisitos 📋

_What things do you need to install the software and how to install them._

```
**Supported:** Ubuntu 20.04 & Debian Buster (10)
**Experimental:** Windows 10, Arch Linux
`ROS Noetic` : <http://wiki.ros.org/noetic/Installation>
`Catkin_make ROS Enviroment` : <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>
`Arduino` : <https://www.arduino.cc/en/software>
python => 3.6
```
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

## Running testing with ROS Enviroment  _ceres Branch in git_⚙️

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


## Contribuyendo 🖇️

Por favor lee el [CONTRIBUTING.md](https://gist.github.com/villanuevand/xxxxxx) para detalles de nuestro código de conducta, y el proceso para enviarnos pull requests.

## GIDAM 📖

You can know more about the research group [GIDAM](https://www.umng.edu.co/sedes/bogota/facultad-de-ingenieria/centro-de-investigacion/gidam)


## Autores ✒️


* **Leonardo Enrique Solaque** - *Researcher* - [Leonardo](https://github)
* **Alexandra Velasco Vivas** - *Researcher* - [Alexandra](https://www.linkedin.com/in/alexandra-velasco-vivas-2446a62a/)
* **Adriana Riveros Guevara** - *Researcher* - [Alexandra](https://w)

* **Adrien Legrand** - *Contribution with thesis* - [Adrien](https://www.linkedin.com/in/adrienlegrand/)
* **Guillermo Sanchez Herrera** - *research assistant* - [Guillermo](https://www.linkedin.com/in/datacloudgui/)
 

## Licencia 📄

This project is under the License - see the file [LICENSE.md] (LICENSE.md) for details

## Expresiones de Gratitud 🎁

* Comenta a otros sobre este proyecto 📢
* Invita una cerveza 🍺 o un café ☕ a alguien del equipo. 
* Da las gracias públicamente 🤓.



---
⌨️ con ❤️ por [Villanuevand](https://github.com/Villanuevand) 😊

