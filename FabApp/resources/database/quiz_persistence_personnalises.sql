-- FABOS - données quiz persistantes et personnalisées
-- Aucun CREATE, ALTER ou DROP : ce script ne change pas le schéma ni les entités.
-- Il ajoute uniquement des FORMATION/SECTION/QUIZ/QUESTION/CHOIX internes.
-- Exécuter ce script dans la base configurée par DATABASE_URL (aucun nom de base imposé).
SET NAMES utf8mb4 COLLATE utf8mb4_unicode_ci;
START TRANSACTION;

-- Une seconde étape pédagogique est ajoutée aux formations visibles.
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation imprimante 3D' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) 
SELECT @parent_id, 'Mise en pratique et autonomie', 'Exercice guidé, vérification du poste et validation des gestes avant utilisation autonome.', NULL, 2, CURRENT_TIMESTAMP 
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);

SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation découpe laser CO2' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) 
SELECT @parent_id, 'Mise en pratique et autonomie', 'Exercice guidé, vérification du poste et validation des gestes avant utilisation autonome.', NULL, 2, CURRENT_TIMESTAMP 
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);

SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation soudure électronique' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) 
SELECT @parent_id, 'Mise en pratique et autonomie', 'Exercice guidé, vérification du poste et validation des gestes avant utilisation autonome.', NULL, 2, CURRENT_TIMESTAMP 
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);

SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation découpe vinyle' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) 
SELECT @parent_id, 'Mise en pratique et autonomie', 'Exercice guidé, vérification du poste et validation des gestes avant utilisation autonome.', NULL, 2, CURRENT_TIMESTAMP 
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);

SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation fraiseuse CNC' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) 
SELECT @parent_id, 'Mise en pratique et autonomie', 'Exercice guidé, vérification du poste et validation des gestes avant utilisation autonome.', NULL, 2, CURRENT_TIMESTAMP 
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);

SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation oscilloscope numérique' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) 
SELECT @parent_id, 'Mise en pratique et autonomie', 'Exercice guidé, vérification du poste et validation des gestes avant utilisation autonome.', NULL, 2, CURRENT_TIMESTAMP 
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);

SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation brodeuse numérique' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) 
SELECT @parent_id, 'Mise en pratique et autonomie', 'Exercice guidé, vérification du poste et validation des gestes avant utilisation autonome.', NULL, 2, CURRENT_TIMESTAMP 
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);

-- Imprimante 3D test / Sécurité et préparation
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'printer-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation imprimante 3D' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Imprimante 3D test · Sécurité et préparation';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Imprimante 3D test.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Sécurité et préparation · Imprimante 3D test', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Avant d’utiliser Imprimante 3D test, quelle vérification administrative est indispensable ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Avant d’utiliser Imprimante 3D test, quelle vérification administrative est indispensable ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel est le risque principal à prendre en compte sur Imprimante 3D test ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quel est le risque principal à prendre en compte sur Imprimante 3D test ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles vérifications préparer avant le démarrage de Imprimante 3D test ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles vérifications préparer avant le démarrage de Imprimante 3D test ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel choix de matière ou de consommable est correct pour Imprimante 3D test ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Quel choix de matière ou de consommable est correct pour Imprimante 3D test ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel comportement est interdit avec Imprimante 3D test ?', 'single', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quel comportement est interdit avec Imprimante 3D test ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'En cas de danger immédiat sur Imprimante 3D test, quelles actions sont adaptées ?', 'multiple', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'En cas de danger immédiat sur Imprimante 3D test, quelles actions sont adaptées ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que la machine est disponible et que mon autorisation ou ma formation est valide', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Utiliser le compte d’un autre membre', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer un statut de maintenance', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lancer la machine avant de réserver', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'la buse et le plateau chauds ainsi que les axes en mouvement', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur de l’interface', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nom du fichier uniquement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La luminosité de la salle', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'vérifier le profil de tranchage PLA et la propreté du plateau', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dégager la zone de travail et repérer l’arrêt normal ou d’urgence', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Retirer les protections', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Désactiver les alertes de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'un filament PLA propre et compatible avec le profil choisi', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'N’importe quel matériau trouvé dans l’atelier', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un matériau sans étiquette', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un consommable endommagé pour éviter le gaspillage', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'placer un outil ou une main dans la zone mobile pendant l’impression', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lire les consignes affichées', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Demander l’aide du staff en cas de doute', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier la disponibilité de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter la machine avec la commande prévue', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Éloigner les personnes du danger et prévenir un responsable', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Continuer le cycle pour ne pas perdre le travail', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Masquer l’incident dans FabOS', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Imprimante 3D test / Utilisation et incidents
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'printer-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation imprimante 3D' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Imprimante 3D test · Utilisation et incidents';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Imprimante 3D test.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Utilisation et incidents · Imprimante 3D test', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pendant l’utilisation de Imprimante 3D test, quel élément doit être surveillé en priorité ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pendant l’utilisation de Imprimante 3D test, quel élément doit être surveillé en priorité ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelle situation indique qu’il faut intervenir ou arrêter Imprimante 3D test ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelle situation indique qu’il faut intervenir ou arrêter Imprimante 3D test ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles attitudes sont correctes pendant le fonctionnement de Imprimante 3D test ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles attitudes sont correctes pendant le fonctionnement de Imprimante 3D test ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Comment effectuer un arrêt sûr de Imprimante 3D test ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Comment effectuer un arrêt sûr de Imprimante 3D test ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions terminent correctement une session sur Imprimante 3D test ?', 'multiple', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quelles actions terminent correctement une session sur Imprimante 3D test ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel incident doit être enregistré ou signalé après l’utilisation de Imprimante 3D test ?', 'single', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'Quel incident doit être enregistré ou signalé après l’utilisation de Imprimante 3D test ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'la première couche, l’adhérence et l’écoulement régulier du filament', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement l’heure', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le téléphone de l’utilisateur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du bouton de réservation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'un décollement du plateau ou un début de bouchage de buse', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un fonctionnement conforme au test', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un résultat prévu par la procédure', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La fin normale du programme', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Rester attentif aux bruits, odeurs et mouvements inhabituels', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Respecter les protections et la procédure de la machine', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Quitter la zone lorsqu’une surveillance est obligatoire', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Modifier les réglages de maintenance sans autorisation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'arrêter l’impression depuis l’interface puis attendre l’immobilisation des axes', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couper brutalement tous les câbles', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bloquer manuellement les axes', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Attendre que quelqu’un d’autre remarque le problème', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'laisser refroidir le plateau, retirer la pièce sans forcer et nettoyer les résidus', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que le poste est disponible et sûr pour l’utilisateur suivant', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser les déchets sur le poste', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Cacher les défauts observés', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'signaler une buse bouchée, un plateau endommagé ou un bruit anormal', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Aucun incident ne doit être signalé', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Seulement un changement de couleur de l’écran', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement si la machine ne démarre plus du tout', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Ultimaker S5 / Sécurité et préparation
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'ultimaker-s5-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation imprimante 3D' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Ultimaker S5 · Sécurité et préparation';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Ultimaker S5.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Sécurité et préparation · Ultimaker S5', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Avant d’utiliser Ultimaker S5, quelle vérification administrative est indispensable ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Avant d’utiliser Ultimaker S5, quelle vérification administrative est indispensable ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel est le risque principal à prendre en compte sur Ultimaker S5 ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quel est le risque principal à prendre en compte sur Ultimaker S5 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles vérifications préparer avant le démarrage de Ultimaker S5 ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles vérifications préparer avant le démarrage de Ultimaker S5 ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel choix de matière ou de consommable est correct pour Ultimaker S5 ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Quel choix de matière ou de consommable est correct pour Ultimaker S5 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel comportement est interdit avec Ultimaker S5 ?', 'single', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quel comportement est interdit avec Ultimaker S5 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'En cas de danger immédiat sur Ultimaker S5, quelles actions sont adaptées ?', 'multiple', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'En cas de danger immédiat sur Ultimaker S5, quelles actions sont adaptées ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que la machine est disponible et que mon autorisation ou ma formation est valide', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Utiliser le compte d’un autre membre', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer un statut de maintenance', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lancer la machine avant de réserver', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'les print cores chauds, le plateau chauffant et les déplacements rapides de la tête', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur de l’interface', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nom du fichier uniquement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La luminosité de la salle', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'contrôler les print cores AA/BB, les matériaux et le profil de double extrusion', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dégager la zone de travail et repérer l’arrêt normal ou d’urgence', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Retirer les protections', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Désactiver les alertes de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'les matériaux compatibles déclarés dans Cura, notamment PLA et support PVA', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'N’importe quel matériau trouvé dans l’atelier', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un matériau sans étiquette', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un consommable endommagé pour éviter le gaspillage', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'monter un print core incompatible ou tirer sur le filament pendant le chargement', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lire les consignes affichées', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Demander l’aide du staff en cas de doute', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier la disponibilité de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter la machine avec la commande prévue', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Éloigner les personnes du danger et prévenir un responsable', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Continuer le cycle pour ne pas perdre le travail', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Masquer l’incident dans FabOS', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Ultimaker S5 / Utilisation et incidents
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'ultimaker-s5-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation imprimante 3D' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Ultimaker S5 · Utilisation et incidents';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Ultimaker S5.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Utilisation et incidents · Ultimaker S5', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pendant l’utilisation de Ultimaker S5, quel élément doit être surveillé en priorité ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pendant l’utilisation de Ultimaker S5, quel élément doit être surveillé en priorité ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelle situation indique qu’il faut intervenir ou arrêter Ultimaker S5 ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelle situation indique qu’il faut intervenir ou arrêter Ultimaker S5 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles attitudes sont correctes pendant le fonctionnement de Ultimaker S5 ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles attitudes sont correctes pendant le fonctionnement de Ultimaker S5 ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Comment effectuer un arrêt sûr de Ultimaker S5 ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Comment effectuer un arrêt sûr de Ultimaker S5 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions terminent correctement une session sur Ultimaker S5 ?', 'multiple', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quelles actions terminent correctement une session sur Ultimaker S5 ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel incident doit être enregistré ou signalé après l’utilisation de Ultimaker S5 ?', 'single', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'Quel incident doit être enregistré ou signalé après l’utilisation de Ultimaker S5 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'les premières couches, la purge des deux buses et l’adhérence du support soluble', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement l’heure', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le téléphone de l’utilisateur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du bouton de réservation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'un claquement d’extrudeur, une sous-extrusion ou un mélange anormal des matériaux', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un fonctionnement conforme au test', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un résultat prévu par la procédure', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La fin normale du programme', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Rester attentif aux bruits, odeurs et mouvements inhabituels', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Respecter les protections et la procédure de la machine', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Quitter la zone lorsqu’une surveillance est obligatoire', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Modifier les réglages de maintenance sans autorisation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'utiliser la commande d’abandon de l’Ultimaker et attendre le retour en position sûre', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couper brutalement tous les câbles', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bloquer manuellement les axes', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Attendre que quelqu’un d’autre remarque le problème', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'laisser refroidir, retirer le plateau avec précaution et nettoyer les print cores selon la procédure', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que le poste est disponible et sûr pour l’utilisateur suivant', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser les déchets sur le poste', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Cacher les défauts observés', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'signaler un print core obstrué, un défaut de feeder ou une fuite de matière', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Aucun incident ne doit être signalé', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Seulement un changement de couleur de l’écran', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement si la machine ne démarre plus du tout', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Prusa MK3S / Sécurité et préparation
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'prusa-mk3s-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation imprimante 3D' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Prusa MK3S · Sécurité et préparation';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Prusa MK3S.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Sécurité et préparation · Prusa MK3S', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Avant d’utiliser Prusa MK3S, quelle vérification administrative est indispensable ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Avant d’utiliser Prusa MK3S, quelle vérification administrative est indispensable ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel est le risque principal à prendre en compte sur Prusa MK3S ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quel est le risque principal à prendre en compte sur Prusa MK3S ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles vérifications préparer avant le démarrage de Prusa MK3S ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles vérifications préparer avant le démarrage de Prusa MK3S ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel choix de matière ou de consommable est correct pour Prusa MK3S ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Quel choix de matière ou de consommable est correct pour Prusa MK3S ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel comportement est interdit avec Prusa MK3S ?', 'single', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quel comportement est interdit avec Prusa MK3S ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'En cas de danger immédiat sur Prusa MK3S, quelles actions sont adaptées ?', 'multiple', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'En cas de danger immédiat sur Prusa MK3S, quelles actions sont adaptées ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que la machine est disponible et que mon autorisation ou ma formation est valide', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Utiliser le compte d’un autre membre', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer un statut de maintenance', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lancer la machine avant de réserver', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'la buse chaude, le plateau chauffant et le déplacement du chariot sur ses axes', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur de l’interface', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nom du fichier uniquement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La luminosité de la salle', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'vérifier la feuille ressort, le profil de filament et la qualité de la première couche', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dégager la zone de travail et repérer l’arrêt normal ou d’urgence', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Retirer les protections', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Désactiver les alertes de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'un filament compatible avec la feuille et la température configurées', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'N’importe quel matériau trouvé dans l’atelier', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un matériau sans étiquette', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un consommable endommagé pour éviter le gaspillage', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'décoller une pièce pendant le mouvement ou forcer sur les axes de la machine', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lire les consignes affichées', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Demander l’aide du staff en cas de doute', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier la disponibilité de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter la machine avec la commande prévue', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Éloigner les personnes du danger et prévenir un responsable', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Continuer le cycle pour ne pas perdre le travail', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Masquer l’incident dans FabOS', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Prusa MK3S / Utilisation et incidents
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'prusa-mk3s-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation imprimante 3D' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Prusa MK3S · Utilisation et incidents';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Prusa MK3S.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Utilisation et incidents · Prusa MK3S', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pendant l’utilisation de Prusa MK3S, quel élément doit être surveillé en priorité ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pendant l’utilisation de Prusa MK3S, quel élément doit être surveillé en priorité ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelle situation indique qu’il faut intervenir ou arrêter Prusa MK3S ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelle situation indique qu’il faut intervenir ou arrêter Prusa MK3S ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles attitudes sont correctes pendant le fonctionnement de Prusa MK3S ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles attitudes sont correctes pendant le fonctionnement de Prusa MK3S ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Comment effectuer un arrêt sûr de Prusa MK3S ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Comment effectuer un arrêt sûr de Prusa MK3S ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions terminent correctement une session sur Prusa MK3S ?', 'multiple', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quelles actions terminent correctement une session sur Prusa MK3S ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel incident doit être enregistré ou signalé après l’utilisation de Prusa MK3S ?', 'single', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'Quel incident doit être enregistré ou signalé après l’utilisation de Prusa MK3S ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'la première couche, le capteur de filament et l’absence de formation de spaghetti', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement l’heure', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le téléphone de l’utilisateur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du bouton de réservation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'une première couche trop écrasée, trop haute ou une impression qui se décolle', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un fonctionnement conforme au test', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un résultat prévu par la procédure', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La fin normale du programme', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Rester attentif aux bruits, odeurs et mouvements inhabituels', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Respecter les protections et la procédure de la machine', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Quitter la zone lorsqu’une surveillance est obligatoire', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Modifier les réglages de maintenance sans autorisation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'arrêter l’impression avec la molette puis attendre l’arrêt et le dégagement de la tête', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couper brutalement tous les câbles', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bloquer manuellement les axes', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Attendre que quelqu’un d’autre remarque le problème', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'laisser refroidir la feuille, la fléchir hors de la machine et nettoyer sa surface', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que le poste est disponible et sûr pour l’utilisateur suivant', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser les déchets sur le poste', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Cacher les défauts observés', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'signaler une feuille abîmée, une buse obstruée ou un axe qui force', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Aucun incident ne doit être signalé', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Seulement un changement de couleur de l’écran', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement si la machine ne démarre plus du tout', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Découpeuse laser CO2 / Sécurité et préparation
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'laser-co2-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation découpe laser CO2' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Découpeuse laser CO2 · Sécurité et préparation';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Découpeuse laser CO2.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Sécurité et préparation · Découpeuse laser CO2', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Avant d’utiliser Découpeuse laser CO2, quelle vérification administrative est indispensable ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Avant d’utiliser Découpeuse laser CO2, quelle vérification administrative est indispensable ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel est le risque principal à prendre en compte sur Découpeuse laser CO2 ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quel est le risque principal à prendre en compte sur Découpeuse laser CO2 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles vérifications préparer avant le démarrage de Découpeuse laser CO2 ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles vérifications préparer avant le démarrage de Découpeuse laser CO2 ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel choix de matière ou de consommable est correct pour Découpeuse laser CO2 ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Quel choix de matière ou de consommable est correct pour Découpeuse laser CO2 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel comportement est interdit avec Découpeuse laser CO2 ?', 'single', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quel comportement est interdit avec Découpeuse laser CO2 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'En cas de danger immédiat sur Découpeuse laser CO2, quelles actions sont adaptées ?', 'multiple', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'En cas de danger immédiat sur Découpeuse laser CO2, quelles actions sont adaptées ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que la machine est disponible et que mon autorisation ou ma formation est valide', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Utiliser le compte d’un autre membre', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer un statut de maintenance', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lancer la machine avant de réserver', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'le risque d’incendie, les fumées et le rayonnement confiné dans la machine', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur de l’interface', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nom du fichier uniquement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La luminosité de la salle', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'vérifier le matériau, l’extraction, le fichier et la présence des moyens d’arrêt', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dégager la zone de travail et repérer l’arrêt normal ou d’urgence', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Retirer les protections', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Désactiver les alertes de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'du bois, du carton ou de l’acrylique explicitement validé par le FabLab', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'N’importe quel matériau trouvé dans l’atelier', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un matériau sans étiquette', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un consommable endommagé pour éviter le gaspillage', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'découper du PVC ou un matériau inconnu sans validation', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lire les consignes affichées', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Demander l’aide du staff en cas de doute', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier la disponibilité de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter la machine avec la commande prévue', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Éloigner les personnes du danger et prévenir un responsable', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Continuer le cycle pour ne pas perdre le travail', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Masquer l’incident dans FabOS', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Découpeuse laser CO2 / Utilisation et incidents
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'laser-co2-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation découpe laser CO2' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Découpeuse laser CO2 · Utilisation et incidents';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Découpeuse laser CO2.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Utilisation et incidents · Découpeuse laser CO2', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pendant l’utilisation de Découpeuse laser CO2, quel élément doit être surveillé en priorité ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pendant l’utilisation de Découpeuse laser CO2, quel élément doit être surveillé en priorité ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelle situation indique qu’il faut intervenir ou arrêter Découpeuse laser CO2 ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelle situation indique qu’il faut intervenir ou arrêter Découpeuse laser CO2 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles attitudes sont correctes pendant le fonctionnement de Découpeuse laser CO2 ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles attitudes sont correctes pendant le fonctionnement de Découpeuse laser CO2 ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Comment effectuer un arrêt sûr de Découpeuse laser CO2 ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Comment effectuer un arrêt sûr de Découpeuse laser CO2 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions terminent correctement une session sur Découpeuse laser CO2 ?', 'multiple', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quelles actions terminent correctement une session sur Découpeuse laser CO2 ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel incident doit être enregistré ou signalé après l’utilisation de Découpeuse laser CO2 ?', 'single', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'Quel incident doit être enregistré ou signalé après l’utilisation de Découpeuse laser CO2 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'la flamme, la fumée, le bruit de l’extraction et le déplacement de la tête', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement l’heure', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le téléphone de l’utilisateur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du bouton de réservation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'une flamme persistante, une fumée excessive ou une découpe qui ne traverse pas', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un fonctionnement conforme au test', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un résultat prévu par la procédure', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La fin normale du programme', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Rester attentif aux bruits, odeurs et mouvements inhabituels', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Respecter les protections et la procédure de la machine', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Quitter la zone lorsqu’une surveillance est obligatoire', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Modifier les réglages de maintenance sans autorisation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'arrêter le tir, conserver l’extraction si possible et appliquer la procédure incendie', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couper brutalement tous les câbles', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bloquer manuellement les axes', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Attendre que quelqu’un d’autre remarque le problème', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'attendre, vérifier l’absence de braise puis retirer les chutes et nettoyer la grille', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que le poste est disponible et sûr pour l’utilisateur suivant', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser les déchets sur le poste', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Cacher les défauts observés', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'signaler tout départ de feu, matériau douteux ou défaut d’extraction', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Aucun incident ne doit être signalé', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Seulement un changement de couleur de l’écran', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement si la machine ne démarre plus du tout', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Station de soudure électronique / Sécurité et préparation
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'station-soudure-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation soudure électronique' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Station de soudure électronique · Sécurité et préparation';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Station de soudure électronique.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Sécurité et préparation · Station de soudure électronique', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Avant d’utiliser Station de soudure électronique, quelle vérification administrative est indispensable ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Avant d’utiliser Station de soudure électronique, quelle vérification administrative est indispensable ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel est le risque principal à prendre en compte sur Station de soudure électronique ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quel est le risque principal à prendre en compte sur Station de soudure électronique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles vérifications préparer avant le démarrage de Station de soudure électronique ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles vérifications préparer avant le démarrage de Station de soudure électronique ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel choix de matière ou de consommable est correct pour Station de soudure électronique ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Quel choix de matière ou de consommable est correct pour Station de soudure électronique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel comportement est interdit avec Station de soudure électronique ?', 'single', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quel comportement est interdit avec Station de soudure électronique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'En cas de danger immédiat sur Station de soudure électronique, quelles actions sont adaptées ?', 'multiple', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'En cas de danger immédiat sur Station de soudure électronique, quelles actions sont adaptées ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que la machine est disponible et que mon autorisation ou ma formation est valide', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Utiliser le compte d’un autre membre', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer un statut de maintenance', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lancer la machine avant de réserver', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'la panne brûlante, les fumées de flux et les projections d’étain', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur de l’interface', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nom du fichier uniquement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La luminosité de la salle', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'contrôler la température, le support du fer, l’extraction et l’état des câbles', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dégager la zone de travail et repérer l’arrêt normal ou d’urgence', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Retirer les protections', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Désactiver les alertes de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'de l’étain et des composants autorisés, avec un poste propre et ventilé', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'N’importe quel matériau trouvé dans l’atelier', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un matériau sans étiquette', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un consommable endommagé pour éviter le gaspillage', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'laisser le fer chaud hors de son support ou travailler près de liquides', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lire les consignes affichées', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Demander l’aide du staff en cas de doute', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier la disponibilité de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter la machine avec la commande prévue', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Éloigner les personnes du danger et prévenir un responsable', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Continuer le cycle pour ne pas perdre le travail', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Masquer l’incident dans FabOS', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Station de soudure électronique / Utilisation et incidents
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'station-soudure-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation soudure électronique' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Station de soudure électronique · Utilisation et incidents';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Station de soudure électronique.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Utilisation et incidents · Station de soudure électronique', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pendant l’utilisation de Station de soudure électronique, quel élément doit être surveillé en priorité ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pendant l’utilisation de Station de soudure électronique, quel élément doit être surveillé en priorité ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelle situation indique qu’il faut intervenir ou arrêter Station de soudure électronique ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelle situation indique qu’il faut intervenir ou arrêter Station de soudure électronique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles attitudes sont correctes pendant le fonctionnement de Station de soudure électronique ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles attitudes sont correctes pendant le fonctionnement de Station de soudure électronique ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Comment effectuer un arrêt sûr de Station de soudure électronique ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Comment effectuer un arrêt sûr de Station de soudure électronique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions terminent correctement une session sur Station de soudure électronique ?', 'multiple', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quelles actions terminent correctement une session sur Station de soudure électronique ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel incident doit être enregistré ou signalé après l’utilisation de Station de soudure électronique ?', 'single', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'Quel incident doit être enregistré ou signalé après l’utilisation de Station de soudure électronique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'la mouillabilité de la soudure, la température et l’aspiration des fumées', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement l’heure', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le téléphone de l’utilisateur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du bouton de réservation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'une soudure froide, un pont d’étain ou un composant surchauffé', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un fonctionnement conforme au test', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un résultat prévu par la procédure', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La fin normale du programme', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Rester attentif aux bruits, odeurs et mouvements inhabituels', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Respecter les protections et la procédure de la machine', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Quitter la zone lorsqu’une surveillance est obligatoire', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Modifier les réglages de maintenance sans autorisation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'reposer le fer, couper l’alimentation et sécuriser immédiatement le montage', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couper brutalement tous les câbles', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bloquer manuellement les axes', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Attendre que quelqu’un d’autre remarque le problème', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'étamer légèrement la panne, couper le poste, attendre le refroidissement et ranger', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que le poste est disponible et sûr pour l’utilisateur suivant', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser les déchets sur le poste', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Cacher les défauts observés', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'signaler un câble endommagé, une panne instable ou une brûlure', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Aucun incident ne doit être signalé', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Seulement un changement de couleur de l’écran', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement si la machine ne démarre plus du tout', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Découpeuse vinyle / Sécurité et préparation
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'vinyl-cutter-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation découpe vinyle' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Découpeuse vinyle · Sécurité et préparation';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Découpeuse vinyle.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Sécurité et préparation · Découpeuse vinyle', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Avant d’utiliser Découpeuse vinyle, quelle vérification administrative est indispensable ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Avant d’utiliser Découpeuse vinyle, quelle vérification administrative est indispensable ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel est le risque principal à prendre en compte sur Découpeuse vinyle ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quel est le risque principal à prendre en compte sur Découpeuse vinyle ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles vérifications préparer avant le démarrage de Découpeuse vinyle ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles vérifications préparer avant le démarrage de Découpeuse vinyle ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel choix de matière ou de consommable est correct pour Découpeuse vinyle ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Quel choix de matière ou de consommable est correct pour Découpeuse vinyle ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel comportement est interdit avec Découpeuse vinyle ?', 'single', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quel comportement est interdit avec Découpeuse vinyle ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'En cas de danger immédiat sur Découpeuse vinyle, quelles actions sont adaptées ?', 'multiple', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'En cas de danger immédiat sur Découpeuse vinyle, quelles actions sont adaptées ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que la machine est disponible et que mon autorisation ou ma formation est valide', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Utiliser le compte d’un autre membre', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer un statut de maintenance', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lancer la machine avant de réserver', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'la lame, le déplacement rapide du chariot et le pincement par les galets', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur de l’interface', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nom du fichier uniquement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La luminosité de la salle', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'régler la sortie de lame, placer les galets et effectuer une découpe de test', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dégager la zone de travail et repérer l’arrêt normal ou d’urgence', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Retirer les protections', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Désactiver les alertes de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'un vinyle ou un flex compatible, correctement aligné sous les galets', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'N’importe quel matériau trouvé dans l’atelier', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un matériau sans étiquette', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un consommable endommagé pour éviter le gaspillage', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'sortir excessivement la lame ou tenir le support à la main pendant la découpe', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lire les consignes affichées', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Demander l’aide du staff en cas de doute', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier la disponibilité de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter la machine avec la commande prévue', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Éloigner les personnes du danger et prévenir un responsable', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Continuer le cycle pour ne pas perdre le travail', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Masquer l’incident dans FabOS', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Découpeuse vinyle / Utilisation et incidents
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'vinyl-cutter-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation découpe vinyle' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Découpeuse vinyle · Utilisation et incidents';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Découpeuse vinyle.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Utilisation et incidents · Découpeuse vinyle', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pendant l’utilisation de Découpeuse vinyle, quel élément doit être surveillé en priorité ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pendant l’utilisation de Découpeuse vinyle, quel élément doit être surveillé en priorité ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelle situation indique qu’il faut intervenir ou arrêter Découpeuse vinyle ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelle situation indique qu’il faut intervenir ou arrêter Découpeuse vinyle ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles attitudes sont correctes pendant le fonctionnement de Découpeuse vinyle ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles attitudes sont correctes pendant le fonctionnement de Découpeuse vinyle ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Comment effectuer un arrêt sûr de Découpeuse vinyle ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Comment effectuer un arrêt sûr de Découpeuse vinyle ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions terminent correctement une session sur Découpeuse vinyle ?', 'multiple', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quelles actions terminent correctement une session sur Découpeuse vinyle ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel incident doit être enregistré ou signalé après l’utilisation de Découpeuse vinyle ?', 'single', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'Quel incident doit être enregistré ou signalé après l’utilisation de Découpeuse vinyle ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'l’entraînement droit du rouleau, le bruit de la lame et le décollement éventuel du support', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement l’heure', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le téléphone de l’utilisateur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du bouton de réservation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'un vinyle arraché, une découpe incomplète ou un support qui dérive', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un fonctionnement conforme au test', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un résultat prévu par la procédure', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La fin normale du programme', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Rester attentif aux bruits, odeurs et mouvements inhabituels', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Respecter les protections et la procédure de la machine', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Quitter la zone lorsqu’une surveillance est obligatoire', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Modifier les réglages de maintenance sans autorisation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'mettre la découpe en pause puis dégager le support seulement après l’arrêt du chariot', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couper brutalement tous les câbles', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bloquer manuellement les axes', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Attendre que quelqu’un d’autre remarque le problème', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'rétracter la lame, retirer les chutes et ranger les outils d’échenillage', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que le poste est disponible et sûr pour l’utilisateur suivant', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser les déchets sur le poste', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Cacher les défauts observés', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'signaler une lame cassée, un galet défectueux ou un chariot bloqué', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Aucun incident ne doit être signalé', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Seulement un changement de couleur de l’écran', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement si la machine ne démarre plus du tout', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Fraiseuse CNC / Sécurité et préparation
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'cnc-fraiseuse-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation fraiseuse CNC' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Fraiseuse CNC · Sécurité et préparation';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Fraiseuse CNC.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Sécurité et préparation · Fraiseuse CNC', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Avant d’utiliser Fraiseuse CNC, quelle vérification administrative est indispensable ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Avant d’utiliser Fraiseuse CNC, quelle vérification administrative est indispensable ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel est le risque principal à prendre en compte sur Fraiseuse CNC ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quel est le risque principal à prendre en compte sur Fraiseuse CNC ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles vérifications préparer avant le démarrage de Fraiseuse CNC ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles vérifications préparer avant le démarrage de Fraiseuse CNC ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel choix de matière ou de consommable est correct pour Fraiseuse CNC ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Quel choix de matière ou de consommable est correct pour Fraiseuse CNC ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel comportement est interdit avec Fraiseuse CNC ?', 'single', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quel comportement est interdit avec Fraiseuse CNC ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'En cas de danger immédiat sur Fraiseuse CNC, quelles actions sont adaptées ?', 'multiple', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'En cas de danger immédiat sur Fraiseuse CNC, quelles actions sont adaptées ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que la machine est disponible et que mon autorisation ou ma formation est valide', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Utiliser le compte d’un autre membre', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer un statut de maintenance', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lancer la machine avant de réserver', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'la broche tournante, l’éjection de copeaux et le déplacement automatique des axes', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur de l’interface', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nom du fichier uniquement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La luminosité de la salle', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'contrôler le bridage, la fraise, les origines, le parcours CAM et les limites machine', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dégager la zone de travail et repérer l’arrêt normal ou d’urgence', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Retirer les protections', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Désactiver les alertes de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'une matière validée, bridée sur un martyr avec des fixations hors du parcours outil', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'N’importe quel matériau trouvé dans l’atelier', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un matériau sans étiquette', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un consommable endommagé pour éviter le gaspillage', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'tenir la pièce à la main ou entrer dans la zone de travail pendant la rotation', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lire les consignes affichées', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Demander l’aide du staff en cas de doute', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier la disponibilité de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter la machine avec la commande prévue', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Éloigner les personnes du danger et prévenir un responsable', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Continuer le cycle pour ne pas perdre le travail', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Masquer l’incident dans FabOS', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Fraiseuse CNC / Utilisation et incidents
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'cnc-fraiseuse-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation fraiseuse CNC' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Fraiseuse CNC · Utilisation et incidents';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Fraiseuse CNC.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Utilisation et incidents · Fraiseuse CNC', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pendant l’utilisation de Fraiseuse CNC, quel élément doit être surveillé en priorité ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pendant l’utilisation de Fraiseuse CNC, quel élément doit être surveillé en priorité ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelle situation indique qu’il faut intervenir ou arrêter Fraiseuse CNC ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelle situation indique qu’il faut intervenir ou arrêter Fraiseuse CNC ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles attitudes sont correctes pendant le fonctionnement de Fraiseuse CNC ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles attitudes sont correctes pendant le fonctionnement de Fraiseuse CNC ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Comment effectuer un arrêt sûr de Fraiseuse CNC ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Comment effectuer un arrêt sûr de Fraiseuse CNC ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions terminent correctement une session sur Fraiseuse CNC ?', 'multiple', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quelles actions terminent correctement une session sur Fraiseuse CNC ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel incident doit être enregistré ou signalé après l’utilisation de Fraiseuse CNC ?', 'single', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'Quel incident doit être enregistré ou signalé après l’utilisation de Fraiseuse CNC ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'les vibrations, le bruit de coupe, l’aspiration et la tenue du bridage', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement l’heure', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le téléphone de l’utilisateur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du bouton de réservation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'du broutage, une fraise qui chauffe ou un déplacement vers une bride', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un fonctionnement conforme au test', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un résultat prévu par la procédure', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La fin normale du programme', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Rester attentif aux bruits, odeurs et mouvements inhabituels', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Respecter les protections et la procédure de la machine', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Quitter la zone lorsqu’une surveillance est obligatoire', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Modifier les réglages de maintenance sans autorisation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'utiliser l’arrêt adapté, attendre l’immobilisation complète de la broche puis prévenir le staff', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couper brutalement tous les câbles', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bloquer manuellement les axes', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Attendre que quelqu’un d’autre remarque le problème', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'attendre l’arrêt, retirer les copeaux avec les outils prévus et ranger les fraises', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que le poste est disponible et sûr pour l’utilisateur suivant', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser les déchets sur le poste', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Cacher les défauts observés', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'signaler une collision, une fraise cassée, un bridage déplacé ou une origine incorrecte', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Aucun incident ne doit être signalé', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Seulement un changement de couleur de l’écran', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement si la machine ne démarre plus du tout', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Oscilloscope numérique / Sécurité et préparation
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'oscilloscope-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation oscilloscope numérique' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Oscilloscope numérique · Sécurité et préparation';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Oscilloscope numérique.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Sécurité et préparation · Oscilloscope numérique', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Avant d’utiliser Oscilloscope numérique, quelle vérification administrative est indispensable ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Avant d’utiliser Oscilloscope numérique, quelle vérification administrative est indispensable ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel est le risque principal à prendre en compte sur Oscilloscope numérique ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quel est le risque principal à prendre en compte sur Oscilloscope numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles vérifications préparer avant le démarrage de Oscilloscope numérique ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles vérifications préparer avant le démarrage de Oscilloscope numérique ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel choix de matière ou de consommable est correct pour Oscilloscope numérique ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Quel choix de matière ou de consommable est correct pour Oscilloscope numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel comportement est interdit avec Oscilloscope numérique ?', 'single', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quel comportement est interdit avec Oscilloscope numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'En cas de danger immédiat sur Oscilloscope numérique, quelles actions sont adaptées ?', 'multiple', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'En cas de danger immédiat sur Oscilloscope numérique, quelles actions sont adaptées ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que la machine est disponible et que mon autorisation ou ma formation est valide', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Utiliser le compte d’un autre membre', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer un statut de maintenance', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lancer la machine avant de réserver', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'un court-circuit par la masse de la sonde et le dépassement de la tension d’entrée', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur de l’interface', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nom du fichier uniquement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La luminosité de la salle', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'identifier la référence de masse, la tension attendue et l’atténuation x1/x10 de la sonde', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dégager la zone de travail et repérer l’arrêt normal ou d’urgence', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Retirer les protections', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Désactiver les alertes de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'des montages basse tension conformes aux limites indiquées par le FabLab', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'N’importe quel matériau trouvé dans l’atelier', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un matériau sans étiquette', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un consommable endommagé pour éviter le gaspillage', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'mesurer directement le secteur ou déplacer une masse au hasard sur un circuit alimenté', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lire les consignes affichées', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Demander l’aide du staff en cas de doute', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier la disponibilité de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter la machine avec la commande prévue', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Éloigner les personnes du danger et prévenir un responsable', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Continuer le cycle pour ne pas perdre le travail', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Masquer l’incident dans FabOS', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Oscilloscope numérique / Utilisation et incidents
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'oscilloscope-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation oscilloscope numérique' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Oscilloscope numérique · Utilisation et incidents';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Oscilloscope numérique.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Utilisation et incidents · Oscilloscope numérique', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pendant l’utilisation de Oscilloscope numérique, quel élément doit être surveillé en priorité ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pendant l’utilisation de Oscilloscope numérique, quel élément doit être surveillé en priorité ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelle situation indique qu’il faut intervenir ou arrêter Oscilloscope numérique ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelle situation indique qu’il faut intervenir ou arrêter Oscilloscope numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles attitudes sont correctes pendant le fonctionnement de Oscilloscope numérique ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles attitudes sont correctes pendant le fonctionnement de Oscilloscope numérique ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Comment effectuer un arrêt sûr de Oscilloscope numérique ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Comment effectuer un arrêt sûr de Oscilloscope numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions terminent correctement une session sur Oscilloscope numérique ?', 'multiple', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quelles actions terminent correctement une session sur Oscilloscope numérique ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel incident doit être enregistré ou signalé après l’utilisation de Oscilloscope numérique ?', 'single', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'Quel incident doit être enregistré ou signalé après l’utilisation de Oscilloscope numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'l’échelle verticale, la base de temps, le déclenchement et l’écrêtage du signal', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement l’heure', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le téléphone de l’utilisateur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du bouton de réservation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'un signal saturé, instable ou incohérent avec la fréquence attendue', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un fonctionnement conforme au test', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un résultat prévu par la procédure', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La fin normale du programme', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Rester attentif aux bruits, odeurs et mouvements inhabituels', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Respecter les protections et la procédure de la machine', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Quitter la zone lorsqu’une surveillance est obligatoire', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Modifier les réglages de maintenance sans autorisation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'mettre le montage en sécurité avant de déplacer les connexions ou les masses', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couper brutalement tous les câbles', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bloquer manuellement les axes', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Attendre que quelqu’un d’autre remarque le problème', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'couper le montage, retirer les sondes par les connecteurs et les ranger sans les plier', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que le poste est disponible et sûr pour l’utilisateur suivant', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser les déchets sur le poste', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Cacher les défauts observés', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'signaler une sonde endommagée, une odeur anormale ou une entrée surchargée', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Aucun incident ne doit être signalé', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Seulement un changement de couleur de l’écran', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement si la machine ne démarre plus du tout', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Brodeuse numérique / Sécurité et préparation
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'brodeuse-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation brodeuse numérique' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Brodeuse numérique · Sécurité et préparation';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Brodeuse numérique.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=securite;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Sécurité et préparation · Brodeuse numérique', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Avant d’utiliser Brodeuse numérique, quelle vérification administrative est indispensable ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Avant d’utiliser Brodeuse numérique, quelle vérification administrative est indispensable ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel est le risque principal à prendre en compte sur Brodeuse numérique ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quel est le risque principal à prendre en compte sur Brodeuse numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles vérifications préparer avant le démarrage de Brodeuse numérique ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles vérifications préparer avant le démarrage de Brodeuse numérique ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel choix de matière ou de consommable est correct pour Brodeuse numérique ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Quel choix de matière ou de consommable est correct pour Brodeuse numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel comportement est interdit avec Brodeuse numérique ?', 'single', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quel comportement est interdit avec Brodeuse numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'En cas de danger immédiat sur Brodeuse numérique, quelles actions sont adaptées ?', 'multiple', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'En cas de danger immédiat sur Brodeuse numérique, quelles actions sont adaptées ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que la machine est disponible et que mon autorisation ou ma formation est valide', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Utiliser le compte d’un autre membre', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer un statut de maintenance', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lancer la machine avant de réserver', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'l’aiguille, le cadre mobile et les points de pincement pendant les déplacements', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur de l’interface', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nom du fichier uniquement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La luminosité de la salle', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'contrôler l’aiguille, le fil, le stabilisateur, le cadre et les limites du motif', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dégager la zone de travail et repérer l’arrêt normal ou d’urgence', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Retirer les protections', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Désactiver les alertes de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'un textile et un stabilisateur adaptés, tendus sans déformation excessive', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'N’importe quel matériau trouvé dans l’atelier', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un matériau sans étiquette', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un consommable endommagé pour éviter le gaspillage', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'placer les mains dans la zone du cadre ou forcer un cadre mal verrouillé', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lire les consignes affichées', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Demander l’aide du staff en cas de doute', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier la disponibilité de la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter la machine avec la commande prévue', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Éloigner les personnes du danger et prévenir un responsable', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Continuer le cycle pour ne pas perdre le travail', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Masquer l’incident dans FabOS', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

-- Brodeuse numérique / Utilisation et incidents
SET @machine_id = (SELECT id FROM MACHINE WHERE machineToken = 'brodeuse-01' LIMIT 1);
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation brodeuse numérique' AND (categorie IS NULL OR categorie <> 'Quiz interne') ORDER BY id LIMIT 1);
SET @quiz_title = '[FABOS QUIZ] Brodeuse numérique · Utilisation et incidents';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni) 
SELECT parent.badgeId, @quiz_title, 'Évaluation persistante personnalisée pour Brodeuse numérique.', parent.image, 'Quiz interne', parent.niveau, '8 min', 'FabOS', NULL, 'Valider les règles de sécurité et les bons gestes.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;'), 'Quiz en ligne' 
FROM FORMATION parent WHERE parent.id = @parent_id AND @machine_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne');
SET @quiz_formation_id = (SELECT id FROM FORMATION WHERE titre = @quiz_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_MACHINE_ID=', @machine_id, ';FABOS_QUIZ_KEY=pratique;') WHERE id = @quiz_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt) SELECT @quiz_formation_id, 'Utilisation et incidents · Brodeuse numérique', 'Six questions personnalisées. Le meilleur score est conservé dans PROGRESSION.', NULL, 1, CURRENT_TIMESTAMP WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1);
SET @quiz_section_id = (SELECT id FROM SECTION WHERE formationId = @quiz_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @quiz_formation_id, @quiz_section_id, 80 WHERE @quiz_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @quiz_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @quiz_formation_id ORDER BY id LIMIT 1);
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pendant l’utilisation de Brodeuse numérique, quel élément doit être surveillé en priorité ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pendant l’utilisation de Brodeuse numérique, quel élément doit être surveillé en priorité ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelle situation indique qu’il faut intervenir ou arrêter Brodeuse numérique ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelle situation indique qu’il faut intervenir ou arrêter Brodeuse numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles attitudes sont correctes pendant le fonctionnement de Brodeuse numérique ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles attitudes sont correctes pendant le fonctionnement de Brodeuse numérique ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Comment effectuer un arrêt sûr de Brodeuse numérique ?', 'single', 4 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 4);
UPDATE QUESTION SET texte = 'Comment effectuer un arrêt sûr de Brodeuse numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 4;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions terminent correctement une session sur Brodeuse numérique ?', 'multiple', 5 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 5);
UPDATE QUESTION SET texte = 'Quelles actions terminent correctement une session sur Brodeuse numérique ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 5;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel incident doit être enregistré ou signalé après l’utilisation de Brodeuse numérique ?', 'single', 6 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 6);
UPDATE QUESTION SET texte = 'Quel incident doit être enregistré ou signalé après l’utilisation de Brodeuse numérique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 6;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'la tension du fil, les changements de couleur et la liberté de déplacement du cadre', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement l’heure', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le téléphone de l’utilisateur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du bouton de réservation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'une casse de fil, un nid de fil sous le textile ou un motif qui se décale', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un fonctionnement conforme au test', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un résultat prévu par la procédure', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La fin normale du programme', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Rester attentif aux bruits, odeurs et mouvements inhabituels', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Respecter les protections et la procédure de la machine', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Quitter la zone lorsqu’une surveillance est obligatoire', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Modifier les réglages de maintenance sans autorisation', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'mettre la brodeuse en pause, attendre l’arrêt de l’aiguille puis dégager la zone', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couper brutalement tous les câbles', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bloquer manuellement les axes', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Attendre que quelqu’un d’autre remarque le problème', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 4 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'remonter l’aiguille, retirer le cadre, couper les fils et nettoyer les chutes', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que le poste est disponible et sûr pour l’utilisateur suivant', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser les déchets sur le poste', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Cacher les défauts observés', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 5 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'signaler une aiguille cassée, un cadre heurté ou un mécanisme qui force', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Aucun incident ne doit être signalé', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Seulement un changement de couleur de l’écran', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement si la machine ne démarre plus du tout', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 6 LIMIT 1;

COMMIT;

-- Vérification rapide
SELECT COUNT(*) AS quiz_formations_internes FROM FORMATION WHERE categorie = 'Quiz interne';
SELECT COUNT(*) AS quiz_persistants FROM QUIZ q INNER JOIN FORMATION f ON f.id = q.formationId WHERE f.categorie = 'Quiz interne';
SELECT COUNT(*) AS questions_personnalisees FROM QUESTION q INNER JOIN QUIZ z ON z.id = q.quizId INNER JOIN FORMATION f ON f.id = z.formationId WHERE f.categorie = 'Quiz interne';