# Tekbot ROS2 Projects 
Dans le cadre du Tekbot Robotics Challenges 2025, notre équipe a relevé trois défis techniques intensifs, réalisés à raison d’un projet par semaine. Ces travaux ont permis de mettre en pratique nos compétences en développement logiciel, en robotique et en simulation, à travers des outils professionnels comme ROS2, Gazebo et des langages comme Python et C++.

# Projet 1 - Programmation Orientée Objet: Conception d’une classe Robot
Ce premier projet consistait à développer une classe Robot en Python et C++, en appliquant les principes de la programmation orientée objet : encapsulation, héritage et polymorphisme. Nous avons conçu une architecture modulaire avec une méthode move() redéfinie dans plusieurs sous-classes, accompagnée de diagrammes UML détaillant la structure du code.

# Projet 2 - ROS2: Évaluation de données capteurs
Cette partie est basée sur la création de package ROS2 nommé sensor_data_evaluation avec deux nodes:

Un publisher qui génère aléatoirement des données de capteurs (température, humidité, pression) toutes les 0,5 secondes.
Un subscriber qui vérifie si ces données respectent les plages autorisées et envoie des messages dans les logs.
Le tout est orchestré via un fichier de lancement, garantissant une exécution fluide et sans erreurs.

# Projet 3 - Navigation autonome avec Pathfinding
À l’aide de ROS2, Gazebo et RViz2, nous avons développé un système de navigation autonome pour le robot TekBot. L’objectif était d'implémenter un algorithme de pathfinding (A*, Dijkstra, ou RRT) permettant au robot de se déplacer efficacement dans un environnement simulé en évitant les obstacles. La simulation visuelle valide la robustesse de l’algorithme et l’intégration avec ROS2.
