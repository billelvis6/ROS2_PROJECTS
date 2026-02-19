
# ROS2 Projects Repository

Bienvenue dans le dépôt principal des projets ROS2 réalisés par **Bill-Elvis SOMAKOU** Lors de la TEKBOT ROBOTIC CHALLENGE 2025.  
Ce dépôt rassemble plusieurs expérimentations et projets pédagogiques autour de ROS2 Humble, incluant la simulation, la navigation autonome et la communication inter-nodes.



## Contenu du dépôt

1. **Sensor Data Evaluation**
   - Package ROS2 C++ simulant des capteurs environnementaux.
   - Fonctionnalités : publication de données (température, humidité, pression) et évaluation en temps réel via un subscriber.
   - Node publisher + subscriber + fichier de lancement XML.
   

2. **TRC ROSMaster X3 Simulation**
   - Simulation d’un robot X3 dans Gazebo.
   - Navigation autonome avec SLAM, RViz et Nav2.
   - Détection de QR codes pour positionnement et collecte de données.
   - Includes nodes de téléopération, QR detection et mapping.


3. **Autres projets ROS2**
   - Expérimentations sur la commande de servomoteurs.
   - Environnement de test multi-nodes et inter-machine.
   - ROS2 pub/sub avancé, timers et QoS.



##  Objectifs généraux

- Apprendre et maîtriser la communication **Publisher/Subscriber**.
- Développer des nodes ROS2 en **C++ et Python**.
- Comprendre l’intégration **Gazebo + RViz + Nav2**.
- Simuler et contrôler des robots autonomes.
- Déployer des nodes sur plusieurs machines.



##  Installation et prérequis

1. Installer **ROS2 Humble** :  
   [Documentation officielle](https://docs.ros.org/en/humble/Installation.html)

2. Créer et préparer un workspace ROS2 :

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
````

3. Cloner le dépôt :

```bash
cd ~/ros2_ws/src
git clone <URL_DU_DEPOT>
cd ..
colcon build --symlink-install
source install/setup.bash
```



## Commandes utiles

* Lister les topics : `ros2 topic list`
* Écouter un topic : `ros2 topic echo /sensor_data`
* Visualiser les nodes et connexions : `rqt_graph`
* Lancer un projet : `ros2 launch <package> <launch_file>`



##  Perspectives

* Création de messages ROS2 personnalisés.
* Intégration de capteurs réels avec simulation.
* Tests inter-machines pour DDS distribué.
* Amélioration de l’interface RViz et des nodes.
* Développement de scripts Python de lancement et gestion multi-robot.


## 🔗 Ressources

* [Documentation ROS2 Humble](https://docs.ros.org/en/humble/index.html)
* [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
* [Gazebo Tutorials](http://gazebosim.org/tutorials)
* [Nav2 Documentation](https://navigation.ros.org/)


**Auteur :** Bill-Elvis SOMAKOU
**Année :** 2025
**Licence :** MIT


