-- FABOS - formations demo complementaires et liens MACHINE_FORMATION
-- SQL controle et idempotent: aucune structure modifiee, aucun doublon cree.

INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT NULL,
       'Formation découpe laser CO2',
       'Formation pratique pour utiliser la découpeuse laser CO2 du FabLab en sécurité, préparer des fichiers vectoriels propres et choisir les bons paramètres de découpe ou gravure.',
       NULL,
       'Découpe laser',
       2,
       '3h',
       'Équipe FabLab',
       6,
       'Comprendre les risques liés au laser CO2 et à l’extraction. Préparer un fichier vectoriel compatible. Choisir puissance, vitesse et passes selon le matériau. Réaliser une découpe et une gravure de test sous supervision.',
       'Compte FabOS actif. Bases de dessin vectoriel recommandées. Utilisation de matériaux validés par le FabLab uniquement.',
       'Découpeuse laser CO2, chutes de bois et acrylique, poste de préparation vectorielle, fiche paramètres matière, lunettes et consignes sécurité.'
WHERE NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = 'Formation découpe laser CO2');

INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT NULL,
       'Formation soudure électronique',
       'Formation d’initiation au poste de soudure électronique pour assembler, réparer et contrôler des circuits simples dans de bonnes conditions de sécurité.',
       NULL,
       'Électronique',
       1,
       '2h',
       'Équipe FabLab',
       8,
       'Identifier les composants courants et leurs polarités. Régler et utiliser un fer à souder. Réaliser une soudure propre sur PCB. Contrôler une continuité et diagnostiquer une soudure froide.',
       'Aucun prérequis obligatoire. Bases de sécurité atelier et soin du poste de travail demandés.',
       'Station de soudure, étain, support PCB, composants de démonstration, pompe ou tresse à dessouder, multimètre, extraction de fumée.'
WHERE NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = 'Formation soudure électronique');

INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT NULL,
       'Formation découpe vinyle',
       'Formation pour préparer des fichiers de découpe, régler la lame et produire des stickers, pochoirs ou marquages simples avec la découpeuse vinyle.',
       NULL,
       'Découpe vinyle',
       1,
       '1h30',
       'Équipe FabLab',
       8,
       'Préparer un tracé vectoriel fermé. Régler force, vitesse et profondeur de lame. Charger correctement le vinyle. Écheniller et transférer une découpe simple.',
       'Compte FabOS actif. Notions de dessin vectoriel utiles mais non obligatoires.',
       'Découpeuse vinyle, rouleaux de vinyle de test, tapis ou support de coupe, tape de transfert, spatule, outils d’échenillage.'
WHERE NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = 'Formation découpe vinyle');

INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT NULL,
       'Formation fraiseuse CNC',
       'Formation avancée pour utiliser la fraiseuse CNC avec une préparation CAM, un bridage correct et une supervision adaptée aux risques d’usinage.',
       NULL,
       'Usinage CNC',
       3,
       '4h',
       'Équipe FabLab',
       4,
       'Comprendre les risques mécaniques et les zones interdites. Préparer un parcours outil CAM simple. Choisir une fraise et des paramètres de coupe adaptés. Brider une pièce et lancer un usinage supervisé.',
       'Formation sécurité atelier recommandée. Bases de CAO/CAM nécessaires. Validation du projet par un encadrant avant usinage.',
       'Fraiseuse CNC, fraises de démonstration, martyr, brides, clés et palpeur, aspiration copeaux, matière de test, fiche sécurité machine.'
WHERE NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = 'Formation fraiseuse CNC');

INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT NULL,
       'Formation oscilloscope numérique',
       'Formation pour prendre en main l’oscilloscope numérique, effectuer des mesures de signaux basse tension et documenter un diagnostic électronique simple.',
       NULL,
       'Électronique',
       1,
       '2h',
       'Équipe FabLab',
       6,
       'Brancher correctement sondes et masses. Régler base de temps, tension et déclenchement. Mesurer fréquence, amplitude et rapport cyclique. Capturer et interpréter un signal simple.',
       'Bases d’électricité recommandées. Mesures limitées aux signaux basse tension autorisés par le FabLab.',
       'Oscilloscope numérique, sondes, générateur de signal de démonstration, breadboard, câbles, fiche mémo de mesure.'
WHERE NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = 'Formation oscilloscope numérique');

INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT NULL,
       'Formation brodeuse numérique',
       'Formation pour préparer un motif de broderie, choisir un support textile, régler le cadre et produire un premier échantillon propre avec la brodeuse numérique.',
       NULL,
       'Textile',
       2,
       '2h30',
       'Équipe FabLab',
       5,
       'Préparer ou importer un fichier de broderie. Choisir fil, aiguille, stabilisateur et cadre. Positionner correctement le textile. Lancer une broderie de test et gérer les changements de fil.',
       'Compte FabOS actif. Apporter un motif simple ou utiliser le motif de démonstration fourni.',
       'Brodeuse numérique, cadres, fils polyester, aiguilles, stabilisateurs, chutes textile, ciseaux, fiche préparation motif.'
WHERE NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = 'Formation brodeuse numérique');

INSERT INTO MACHINE_FORMATION (machineId, formationId, requiredForAccess, niveauRequis)
SELECT m.id, f.id, 1, f.niveau
FROM MACHINE m
JOIN FORMATION f ON f.titre = 'Formation découpe laser CO2'
WHERE m.machineToken = 'laser-co2-01'
  AND NOT EXISTS (SELECT 1 FROM MACHINE_FORMATION mf WHERE mf.machineId = m.id AND mf.formationId = f.id);

INSERT INTO MACHINE_FORMATION (machineId, formationId, requiredForAccess, niveauRequis)
SELECT m.id, f.id, 1, f.niveau
FROM MACHINE m
JOIN FORMATION f ON f.titre = 'Formation soudure électronique'
WHERE m.machineToken = 'station-soudure-01'
  AND NOT EXISTS (SELECT 1 FROM MACHINE_FORMATION mf WHERE mf.machineId = m.id AND mf.formationId = f.id);

INSERT INTO MACHINE_FORMATION (machineId, formationId, requiredForAccess, niveauRequis)
SELECT m.id, f.id, 1, f.niveau
FROM MACHINE m
JOIN FORMATION f ON f.titre = 'Formation découpe vinyle'
WHERE m.machineToken = 'vinyl-cutter-01'
  AND NOT EXISTS (SELECT 1 FROM MACHINE_FORMATION mf WHERE mf.machineId = m.id AND mf.formationId = f.id);

INSERT INTO MACHINE_FORMATION (machineId, formationId, requiredForAccess, niveauRequis)
SELECT m.id, f.id, 1, f.niveau
FROM MACHINE m
JOIN FORMATION f ON f.titre = 'Formation fraiseuse CNC'
WHERE m.machineToken = 'cnc-fraiseuse-01'
  AND NOT EXISTS (SELECT 1 FROM MACHINE_FORMATION mf WHERE mf.machineId = m.id AND mf.formationId = f.id);

INSERT INTO MACHINE_FORMATION (machineId, formationId, requiredForAccess, niveauRequis)
SELECT m.id, f.id, 1, f.niveau
FROM MACHINE m
JOIN FORMATION f ON f.titre = 'Formation oscilloscope numérique'
WHERE m.machineToken = 'oscilloscope-01'
  AND NOT EXISTS (SELECT 1 FROM MACHINE_FORMATION mf WHERE mf.machineId = m.id AND mf.formationId = f.id);

INSERT INTO MACHINE_FORMATION (machineId, formationId, requiredForAccess, niveauRequis)
SELECT m.id, f.id, 1, f.niveau
FROM MACHINE m
JOIN FORMATION f ON f.titre = 'Formation brodeuse numérique'
WHERE m.machineToken = 'brodeuse-01'
  AND NOT EXISTS (SELECT 1 FROM MACHINE_FORMATION mf WHERE mf.machineId = m.id AND mf.formationId = f.id);
