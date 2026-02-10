# Projet 3 - Navigation autonome 
Contexte et objectifs:
Avec l’essor des robots mobiles autonomes, la capacité à naviguer efficacement dans des environnements complexes et dynamiques est devenue un enjeu majeur. Pour permettre à un robot de se déplacer de manière autonome, il est essentiel de lui fournir un système de navigation capable de planifier des trajectoires optimales tout en évitant les obstacles.

ROS2, combiné au simulateur Gazebo, offre un cadre complet pour développer, tester et valider ces systèmes dans un environnement virtuel proche du réel. Le robot TekBot, équipé de capteurs et d’une configuration prête à l’emploi, constitue une base idéale pour expérimenter et implémenter des algorithmes de pathfinding classiques dans un labyrinthe simulé.

L’objectif principal de ce projet est de créer un système de navigation autonome pour un robot mobile en utilisant:

ROS2 Humble sous Ubuntu 22.04
Gazebo Classic: simulateur 3D permettant de visualiser l'environnement et le robot
RViz: outil de visualisation pour les données et les capteurs ROS2.
SLAM Toolbox: utilisé pour le mapping et la localisation simultanée.
Nav2: framework pour la navigation autonome, intégrant le pathfinding et l'évitement d'obstacles.
