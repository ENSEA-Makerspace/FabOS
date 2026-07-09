-- FABOS - parcours guidés, mini-quiz de sections et quiz bonus
-- Aucun CREATE, ALTER ou DROP : ce script ne modifie ni le schéma ni les entités.
-- Les mini-quiz de section sont stockés dans les tables existantes FORMATION/SECTION/QUIZ/QUESTION/CHOIX.
-- Les résultats utilisent PROGRESSION via des formations internes invisibles dans le catalogue.
SET NAMES utf8mb4 COLLATE utf8mb4_unicode_ci;
START TRANSACTION;

-- ============================================================
-- Formation imprimante 3D
-- ============================================================
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation imprimante 3D' AND (categorie IS NULL OR categorie NOT IN ('Quiz interne', 'Validation physique')) ORDER BY id LIMIT 1);

-- Section 1 : Préparer le fichier et choisir le bon matériau
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Préparer le fichier et choisir le bon matériau', '{"intro":"Une impression fiable commence avant même d''allumer la machine : le modèle, son orientation et le filament doivent être cohérents avec l''usage final.","objectives":["Vérifier un fichier imprimable","Choisir un filament compatible","Anticiper supports et orientation"],"steps":[{"title":"Contrôler le modèle","text":"Vérifiez les dimensions, l''échelle, les parois trop fines et l''absence de géométrie ouverte."},{"title":"Orienter intelligemment","text":"Placez la pièce pour limiter les supports, améliorer la résistance et réduire le temps d''impression."},{"title":"Choisir le profil","text":"Associez le bon type de filament, le diamètre, la buse et le profil validé dans le slicer."}],"callouts":[{"type":"tip","title":"Réflexe FabLab","text":"Une orientation simple vaut souvent mieux qu''une impression plus rapide mais fragile."},{"type":"warning","title":"À éviter","text":"Ne lancez jamais un matériau inconnu ou humide avec un profil choisi au hasard."}]}', NULL, 1, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 1);
UPDATE SECTION SET titre = 'Préparer le fichier et choisir le bon matériau', contenu = '{"intro":"Une impression fiable commence avant même d''allumer la machine : le modèle, son orientation et le filament doivent être cohérents avec l''usage final.","objectives":["Vérifier un fichier imprimable","Choisir un filament compatible","Anticiper supports et orientation"],"steps":[{"title":"Contrôler le modèle","text":"Vérifiez les dimensions, l''échelle, les parois trop fines et l''absence de géométrie ouverte."},{"title":"Orienter intelligemment","text":"Placez la pièce pour limiter les supports, améliorer la résistance et réduire le temps d''impression."},{"title":"Choisir le profil","text":"Associez le bon type de filament, le diamètre, la buse et le profil validé dans le slicer."}],"callouts":[{"type":"tip","title":"Réflexe FabLab","text":"Une orientation simple vaut souvent mieux qu''une impression plus rapide mais fragile."},{"type":"warning","title":"À éviter","text":"Ne lancez jamais un matériau inconnu ou humide avec un profil choisi au hasard."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 1;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 1 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Imprimante 3D · 1';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Imprimante 3D.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Imprimante 3D.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Préparer le fichier et choisir le bon matériau', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Préparer le fichier et choisir le bon matériau', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Avant le tranchage, quelle vérification est prioritaire ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Avant le tranchage, quelle vérification est prioritaire ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quels choix réduisent généralement les supports inutiles ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quels choix réduisent généralement les supports inutiles ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La géométrie, les dimensions et les parois du modèle', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur de l’interface', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nom du dossier Windows', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nombre de fenêtres ouvertes', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Orienter la pièce selon ses surfaces stables', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Placer systématiquement la pièce en diagonale', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Analyser les porte-à-faux avant le tranchage', 1, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Augmenter la vitesse au maximum', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 2 : Préparer le plateau et la machine
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Préparer le plateau et la machine', '{"intro":"Le plateau doit être propre, correctement installé et compatible avec la température du matériau. Quelques secondes de contrôle évitent la majorité des échecs de première couche.","objectives":["Nettoyer sans contaminer","Vérifier buse et plateau","Charger le filament correctement"],"steps":[{"title":"Inspecter la machine","text":"Retirez les anciennes pièces, vérifiez que les axes sont libres et que la buse ne porte pas de dépôt important."},{"title":"Préparer la surface","text":"Nettoyez le plateau avec le produit autorisé et évitez de toucher la zone d''impression avec les doigts."},{"title":"Charger le filament","text":"Contrôlez la bobine, le chemin du filament et l''extrusion régulière avant le lancement."}],"callouts":[{"type":"check","title":"Point de contrôle","text":"Plateau propre, feuille correctement positionnée, filament libre et profil machine correct."},{"type":"warning","title":"Surface chaude","text":"La buse et le plateau peuvent provoquer des brûlures même juste après l''arrêt."}]}', NULL, 2, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);
UPDATE SECTION SET titre = 'Préparer le plateau et la machine', contenu = '{"intro":"Le plateau doit être propre, correctement installé et compatible avec la température du matériau. Quelques secondes de contrôle évitent la majorité des échecs de première couche.","objectives":["Nettoyer sans contaminer","Vérifier buse et plateau","Charger le filament correctement"],"steps":[{"title":"Inspecter la machine","text":"Retirez les anciennes pièces, vérifiez que les axes sont libres et que la buse ne porte pas de dépôt important."},{"title":"Préparer la surface","text":"Nettoyez le plateau avec le produit autorisé et évitez de toucher la zone d''impression avec les doigts."},{"title":"Charger le filament","text":"Contrôlez la bobine, le chemin du filament et l''extrusion régulière avant le lancement."}],"callouts":[{"type":"check","title":"Point de contrôle","text":"Plateau propre, feuille correctement positionnée, filament libre et profil machine correct."},{"type":"warning","title":"Surface chaude","text":"La buse et le plateau peuvent provoquer des brûlures même juste après l''arrêt."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 2;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 2 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Imprimante 3D · 2';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Imprimante 3D.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Imprimante 3D.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Préparer le plateau et la machine', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Préparer le plateau et la machine', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pourquoi faut-il éviter de toucher la zone d’impression du plateau ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pourquoi faut-il éviter de toucher la zone d’impression du plateau ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quels contrôles appartiennent à la préparation de la machine ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quels contrôles appartiennent à la préparation de la machine ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Les traces grasses peuvent réduire l’adhérence', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le plateau change de couleur', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le fichier devient plus lourd', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La machine perd sa connexion réseau', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que les axes sont libres', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'S’assurer que la feuille est bien positionnée', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Forcer la buse à froid', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Désactiver les alertes', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 3 : Lancer et surveiller la première couche
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Lancer et surveiller la première couche', '{"intro":"La première couche est le meilleur indicateur de réussite. Elle doit être régulière, adhérente et suffisamment écrasée sans que la buse ne racle le plateau.","objectives":["Lire une première couche","Interrompre proprement un mauvais départ","Surveiller sans toucher la machine"],"steps":[{"title":"Observer le contour","text":"Regardez le skirt ou la bordure : le filament doit sortir de façon continue et adhérer immédiatement."},{"title":"Contrôler l’écrasement","text":"Des lignes séparées indiquent une buse trop haute ; une trace très fine ou raclée indique une buse trop basse."},{"title":"Décider rapidement","text":"En cas de décollement, de bruit anormal ou de collision, utilisez la commande d''arrêt prévue."}],"callouts":[{"type":"info","title":"Surveillance utile","text":"Restez présent au minimum jusqu''à ce que la première couche soit complètement déposée."},{"type":"warning","title":"Ne jamais faire","text":"Ne mettez pas les doigts près de la buse ou des axes pendant les mouvements."}]}', NULL, 3, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 3);
UPDATE SECTION SET titre = 'Lancer et surveiller la première couche', contenu = '{"intro":"La première couche est le meilleur indicateur de réussite. Elle doit être régulière, adhérente et suffisamment écrasée sans que la buse ne racle le plateau.","objectives":["Lire une première couche","Interrompre proprement un mauvais départ","Surveiller sans toucher la machine"],"steps":[{"title":"Observer le contour","text":"Regardez le skirt ou la bordure : le filament doit sortir de façon continue et adhérer immédiatement."},{"title":"Contrôler l’écrasement","text":"Des lignes séparées indiquent une buse trop haute ; une trace très fine ou raclée indique une buse trop basse."},{"title":"Décider rapidement","text":"En cas de décollement, de bruit anormal ou de collision, utilisez la commande d''arrêt prévue."}],"callouts":[{"type":"info","title":"Surveillance utile","text":"Restez présent au minimum jusqu''à ce que la première couche soit complètement déposée."},{"type":"warning","title":"Ne jamais faire","text":"Ne mettez pas les doigts près de la buse ou des axes pendant les mouvements."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 3;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 3 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Imprimante 3D · 3';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Imprimante 3D.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Imprimante 3D.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Lancer et surveiller la première couche', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Lancer et surveiller la première couche', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel signe indique une première couche probablement trop haute ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Quel signe indique une première couche probablement trop haute ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Que faire si la pièce commence à se décoller dès la première couche ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Que faire si la pièce commence à se décoller dès la première couche ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Les lignes restent séparées et adhèrent mal', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le ventilateur tourne', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'L’écran affiche le temps restant', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La bobine est presque pleine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter avec la commande prévue et corriger la cause', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Appuyer la pièce avec les doigts', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Augmenter brutalement toutes les températures', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser continuer pour voir', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 4 : Fin d’impression et gestion des incidents
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Fin d’impression et gestion des incidents', '{"intro":"Une session se termine par un retrait sans forcer, un contrôle de la machine et un espace laissé propre pour la personne suivante.","objectives":["Retirer une pièce sans abîmer le plateau","Réagir à un bouchage ou à un spaghetti","Ranger et signaler"],"steps":[{"title":"Laisser refroidir","text":"Attendez la température sûre indiquée avant de retirer la feuille ou la pièce."},{"title":"Décoller sans forcer","text":"Utilisez la flexion de la feuille ou l''outil autorisé, sans levier contre la machine."},{"title":"Clore la session","text":"Retirez les déchets, rangez la bobine, nettoyez le plateau et signalez toute anomalie."}],"callouts":[{"type":"tip","title":"Incident visible","text":"Photographiez le défaut avant nettoyage si cela peut aider le staff à diagnostiquer."},{"type":"check","title":"Session terminée","text":"Machine en état, zone propre, consommable rangé et incident déclaré si nécessaire."}]}', NULL, 4, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 4);
UPDATE SECTION SET titre = 'Fin d’impression et gestion des incidents', contenu = '{"intro":"Une session se termine par un retrait sans forcer, un contrôle de la machine et un espace laissé propre pour la personne suivante.","objectives":["Retirer une pièce sans abîmer le plateau","Réagir à un bouchage ou à un spaghetti","Ranger et signaler"],"steps":[{"title":"Laisser refroidir","text":"Attendez la température sûre indiquée avant de retirer la feuille ou la pièce."},{"title":"Décoller sans forcer","text":"Utilisez la flexion de la feuille ou l''outil autorisé, sans levier contre la machine."},{"title":"Clore la session","text":"Retirez les déchets, rangez la bobine, nettoyez le plateau et signalez toute anomalie."}],"callouts":[{"type":"tip","title":"Incident visible","text":"Photographiez le défaut avant nettoyage si cela peut aider le staff à diagnostiquer."},{"type":"check","title":"Session terminée","text":"Machine en état, zone propre, consommable rangé et incident déclaré si nécessaire."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 4;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 4 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Imprimante 3D · 4';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Imprimante 3D.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Imprimante 3D.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Fin d’impression et gestion des incidents', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Fin d’impression et gestion des incidents', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelle est la bonne méthode pour retirer une pièce ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Quelle est la bonne méthode pour retirer une pièce ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions terminent correctement une session ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelles actions terminent correctement une session ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Attendre le refroidissement puis utiliser la méthode prévue pour la feuille', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Faire levier sur le châssis', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Tirer pendant que les axes bougent', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Chauffer au maximum', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nettoyer le plateau', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ranger ou identifier le filament', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Cacher un incident', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser les supports au sol', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Quiz bonus : Défi expert · diagnostiquer un échec d’impression
SET @bonus_title = '[FABOS BONUS] Imprimante 3D · Défi expert';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @bonus_title, 'Quiz bonus facultatif pour Imprimante 3D. Il ne compte pas dans la progression.', parent.image, 'Quiz interne', parent.niveau, '4 min', 'FabOS', NULL, 'Approfondir le diagnostic machine.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;'), 'Quiz bonus'
FROM FORMATION parent WHERE parent.id = @parent_id AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne');
SET @bonus_formation_id = (SELECT id FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Quiz bonus facultatif pour Imprimante 3D. Il ne compte pas dans la progression.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;') WHERE id = @bonus_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @bonus_formation_id, 'Défi expert · diagnostiquer un échec d’impression', 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.', NULL, 1, CURRENT_TIMESTAMP
WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Défi expert · diagnostiquer un échec d’impression', contenu = 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.' WHERE formationId = @bonus_formation_id AND ordre = 1;
SET @bonus_section_id = (SELECT id FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @bonus_formation_id, @bonus_section_id, 67 WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @bonus_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @bonus_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @bonus_section_id, noteMinimale = 67 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Une pièce se décolle progressivement dans les coins. Quelle piste est la plus pertinente ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Une pièce se décolle progressivement dans les coins. Quelle piste est la plus pertinente ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Des couches sont décalées brutalement. Quelles causes sont plausibles ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Des couches sont décalées brutalement. Quelles causes sont plausibles ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Face à une extrusion irrégulière, quel ordre est logique ?', 'single', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Face à une extrusion irrégulière, quel ordre est logique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 3;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Adhérence du plateau, refroidissement ou retrait du matériau', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nom du fichier trop court', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Écran trop lumineux', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Câble réseau trop long', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Obstacle mécanique ou collision', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Courroie ou mouvement d’axe perturbé', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couleur du filament', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Mauvaise police dans le slicer', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier bobine et chemin du filament, puis température et buse', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Changer toutes les pièces immédiatement', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Relancer sans regarder', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ajouter de la colle dans la buse', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;

-- ============================================================
-- Formation découpe laser CO2
-- ============================================================
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation découpe laser CO2' AND (categorie IS NULL OR categorie NOT IN ('Quiz interne', 'Validation physique')) ORDER BY id LIMIT 1);

-- Section 1 : Identifier les risques et les matériaux autorisés
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Identifier les risques et les matériaux autorisés', '{"intro":"Une découpeuse laser concentre chaleur, fumées et mouvement. La première compétence est de savoir ce qui peut entrer dans la machine et ce qui doit rester dehors.","objectives":["Reconnaître les matériaux interdits","Comprendre le risque incendie","Vérifier extraction et accès"],"steps":[{"title":"Identifier la matière","text":"Utilisez uniquement un matériau identifié et validé par le FabLab ; en cas de doute, ne le découpez pas."},{"title":"Repérer les risques","text":"Fumées toxiques, flamme, réflexion et pièces chaudes exigent une surveillance permanente."},{"title":"Contrôler l’environnement","text":"Vérifiez extraction, air assist, extincteur accessible et zone autour de la machine dégagée."}],"callouts":[{"type":"warning","title":"PVC interdit","text":"Le PVC et les matériaux chlorés peuvent produire des gaz toxiques et corrosifs."},{"type":"check","title":"Avant toute découpe","text":"Matière connue, épaisseur mesurée et présence continue de l''opérateur."}]}', NULL, 1, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 1);
UPDATE SECTION SET titre = 'Identifier les risques et les matériaux autorisés', contenu = '{"intro":"Une découpeuse laser concentre chaleur, fumées et mouvement. La première compétence est de savoir ce qui peut entrer dans la machine et ce qui doit rester dehors.","objectives":["Reconnaître les matériaux interdits","Comprendre le risque incendie","Vérifier extraction et accès"],"steps":[{"title":"Identifier la matière","text":"Utilisez uniquement un matériau identifié et validé par le FabLab ; en cas de doute, ne le découpez pas."},{"title":"Repérer les risques","text":"Fumées toxiques, flamme, réflexion et pièces chaudes exigent une surveillance permanente."},{"title":"Contrôler l’environnement","text":"Vérifiez extraction, air assist, extincteur accessible et zone autour de la machine dégagée."}],"callouts":[{"type":"warning","title":"PVC interdit","text":"Le PVC et les matériaux chlorés peuvent produire des gaz toxiques et corrosifs."},{"type":"check","title":"Avant toute découpe","text":"Matière connue, épaisseur mesurée et présence continue de l''opérateur."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 1;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 1 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Découpe laser CO2 · 1';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Découpe laser CO2.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Découpe laser CO2.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Identifier les risques et les matériaux autorisés', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Identifier les risques et les matériaux autorisés', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Que faire avec un plastique dont la composition est inconnue ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Que faire avec un plastique dont la composition est inconnue ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quels éléments doivent être prêts avant le lancement ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quels éléments doivent être prêts avant le lancement ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ne pas le découper et demander validation au staff', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Tester à puissance maximale', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le découper porte ouverte', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le peindre avant usage', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Extraction en fonctionnement', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Moyen d’arrêt accessible', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Machine laissée seule', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Matériau non identifié', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 2 : Préparer le fichier et les paramètres
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Préparer le fichier et les paramètres', '{"intro":"Le laser suit exactement les tracés et paramètres envoyés. Un fichier propre et des réglages issus de la bibliothèque réduisent les brûlures et les passages inutiles.","objectives":["Distinguer découpe et gravure","Nettoyer les doublons","Choisir puissance et vitesse"],"steps":[{"title":"Nettoyer les tracés","text":"Supprimez les doublons, fermez les contours et vérifiez l’échelle réelle du document."},{"title":"Attribuer les opérations","text":"Séparez clairement gravure, marquage et découpe selon la convention du logiciel."},{"title":"Partir d’un réglage validé","text":"Choisissez un profil adapté à la matière et à l’épaisseur puis réalisez un petit test si nécessaire."}],"callouts":[{"type":"tip","title":"Économie de matière","text":"Placez les pièces près d''une zone déjà utilisée tout en gardant une marge sûre."},{"type":"warning","title":"Traits doublés","text":"Deux traits superposés provoquent deux passages et augmentent fortement l''échauffement."}]}', NULL, 2, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);
UPDATE SECTION SET titre = 'Préparer le fichier et les paramètres', contenu = '{"intro":"Le laser suit exactement les tracés et paramètres envoyés. Un fichier propre et des réglages issus de la bibliothèque réduisent les brûlures et les passages inutiles.","objectives":["Distinguer découpe et gravure","Nettoyer les doublons","Choisir puissance et vitesse"],"steps":[{"title":"Nettoyer les tracés","text":"Supprimez les doublons, fermez les contours et vérifiez l’échelle réelle du document."},{"title":"Attribuer les opérations","text":"Séparez clairement gravure, marquage et découpe selon la convention du logiciel."},{"title":"Partir d’un réglage validé","text":"Choisissez un profil adapté à la matière et à l’épaisseur puis réalisez un petit test si nécessaire."}],"callouts":[{"type":"tip","title":"Économie de matière","text":"Placez les pièces près d''une zone déjà utilisée tout en gardant une marge sûre."},{"type":"warning","title":"Traits doublés","text":"Deux traits superposés provoquent deux passages et augmentent fortement l''échauffement."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 2;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 2 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Découpe laser CO2 · 2';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Découpe laser CO2.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Découpe laser CO2.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Préparer le fichier et les paramètres', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Préparer le fichier et les paramètres', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pourquoi faut-il supprimer les traits superposés ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pourquoi faut-il supprimer les traits superposés ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pour choisir les paramètres, quelle méthode est correcte ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Pour choisir les paramètres, quelle méthode est correcte ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ils peuvent provoquer plusieurs passages au même endroit', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ils changent le mot de passe', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ils désactivent l’extraction', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ils agrandissent l’écran', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Partir d’un profil validé pour la matière et l’épaisseur', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Toujours utiliser la puissance maximale', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer l’épaisseur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Copier un réglage d’un matériau inconnu', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 3 : Installer la matière, faire la mise au point et cadrer
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Installer la matière, faire la mise au point et cadrer', '{"intro":"La matière doit être plane, stable et positionnée dans une zone propre. La mise au point et le cadrage évitent une découpe floue ou hors plaque.","objectives":["Positionner sans obstacle","Régler la mise au point","Tester le cadrage"],"steps":[{"title":"Nettoyer le plateau","text":"Retirez les chutes inflammables et vérifiez qu’aucun élément ne dépasse dans la trajectoire."},{"title":"Positionner la matière","text":"Placez-la à plat et utilisez uniquement les moyens de maintien autorisés."},{"title":"Faire focus et cadrage","text":"Réglez la hauteur selon la procédure puis lancez le contour à faible risque avant le travail réel."}],"callouts":[{"type":"info","title":"Cadrage","text":"Le test de contour permet de vérifier l''emprise sans lancer la découpe."},{"type":"warning","title":"Plateau encombré","text":"Les petites chutes accumulées peuvent s''enflammer sous la plaque."}]}', NULL, 3, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 3);
UPDATE SECTION SET titre = 'Installer la matière, faire la mise au point et cadrer', contenu = '{"intro":"La matière doit être plane, stable et positionnée dans une zone propre. La mise au point et le cadrage évitent une découpe floue ou hors plaque.","objectives":["Positionner sans obstacle","Régler la mise au point","Tester le cadrage"],"steps":[{"title":"Nettoyer le plateau","text":"Retirez les chutes inflammables et vérifiez qu’aucun élément ne dépasse dans la trajectoire."},{"title":"Positionner la matière","text":"Placez-la à plat et utilisez uniquement les moyens de maintien autorisés."},{"title":"Faire focus et cadrage","text":"Réglez la hauteur selon la procédure puis lancez le contour à faible risque avant le travail réel."}],"callouts":[{"type":"info","title":"Cadrage","text":"Le test de contour permet de vérifier l''emprise sans lancer la découpe."},{"type":"warning","title":"Plateau encombré","text":"Les petites chutes accumulées peuvent s''enflammer sous la plaque."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 3;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 3 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Découpe laser CO2 · 3';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Découpe laser CO2.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Découpe laser CO2.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Installer la matière, faire la mise au point et cadrer', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Installer la matière, faire la mise au point et cadrer', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'À quoi sert principalement le test de cadrage ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'À quoi sert principalement le test de cadrage ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Que faut-il retirer du plateau avant la découpe ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Que faut-il retirer du plateau avant la découpe ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier que le travail tient dans la matière et la zone prévue', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Augmenter la puissance', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nettoyer automatiquement la lentille', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Mesurer le débit d’air', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Les chutes et éléments inflammables inutiles', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La matière à découper', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le capot de la machine', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Les étiquettes de sécurité', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 4 : Surveiller la découpe et réagir à un incident
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Surveiller la découpe et réagir à un incident', '{"intro":"Une flamme brève peut apparaître, mais une flamme persistante ou une fumée anormale impose un arrêt immédiat selon la procédure.","objectives":["Surveiller en continu","Différencier étincelle et flamme persistante","Arrêter et signaler"],"steps":[{"title":"Rester face à la machine","text":"Observez la zone de travail pendant tout le cycle sans quitter le poste."},{"title":"Détecter l’anormal","text":"Flamme qui suit durablement le faisceau, fumée dense ou bruit inhabituel sont des signaux d’arrêt."},{"title":"Finir proprement","text":"Attendez l’évacuation des fumées, récupérez les pièces avec précaution et nettoyez les chutes."}],"callouts":[{"type":"warning","title":"Flamme persistante","text":"Arrêtez le travail avec la commande prévue et appliquez la procédure incendie du FabLab."},{"type":"check","title":"Après usage","text":"Zone propre, extraction arrêtée selon procédure et anomalie signalée."}]}', NULL, 4, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 4);
UPDATE SECTION SET titre = 'Surveiller la découpe et réagir à un incident', contenu = '{"intro":"Une flamme brève peut apparaître, mais une flamme persistante ou une fumée anormale impose un arrêt immédiat selon la procédure.","objectives":["Surveiller en continu","Différencier étincelle et flamme persistante","Arrêter et signaler"],"steps":[{"title":"Rester face à la machine","text":"Observez la zone de travail pendant tout le cycle sans quitter le poste."},{"title":"Détecter l’anormal","text":"Flamme qui suit durablement le faisceau, fumée dense ou bruit inhabituel sont des signaux d’arrêt."},{"title":"Finir proprement","text":"Attendez l’évacuation des fumées, récupérez les pièces avec précaution et nettoyez les chutes."}],"callouts":[{"type":"warning","title":"Flamme persistante","text":"Arrêtez le travail avec la commande prévue et appliquez la procédure incendie du FabLab."},{"type":"check","title":"Après usage","text":"Zone propre, extraction arrêtée selon procédure et anomalie signalée."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 4;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 4 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Découpe laser CO2 · 4';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Découpe laser CO2.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Découpe laser CO2.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Surveiller la découpe et réagir à un incident', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Surveiller la découpe et réagir à un incident', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Que faire si une flamme persiste pendant la découpe ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Que faire si une flamme persiste pendant la découpe ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quand peut-on quitter la machine ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quand peut-on quitter la machine ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter immédiatement selon la procédure et rester prêt à intervenir', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Augmenter la vitesse sans regarder', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Quitter la salle', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ouvrir immédiatement le capot sans arrêt', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Après la fin du cycle et les vérifications de sécurité', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dès que le fichier est envoyé', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pendant une gravure longue', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Après la première minute', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Quiz bonus : Défi expert · optimiser une découpe propre
SET @bonus_title = '[FABOS BONUS] Découpe laser CO2 · Défi expert';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @bonus_title, 'Quiz bonus facultatif pour Découpe laser CO2. Il ne compte pas dans la progression.', parent.image, 'Quiz interne', parent.niveau, '4 min', 'FabOS', NULL, 'Approfondir le diagnostic machine.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;'), 'Quiz bonus'
FROM FORMATION parent WHERE parent.id = @parent_id AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne');
SET @bonus_formation_id = (SELECT id FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Quiz bonus facultatif pour Découpe laser CO2. Il ne compte pas dans la progression.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;') WHERE id = @bonus_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @bonus_formation_id, 'Défi expert · optimiser une découpe propre', 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.', NULL, 1, CURRENT_TIMESTAMP
WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Défi expert · optimiser une découpe propre', contenu = 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.' WHERE formationId = @bonus_formation_id AND ordre = 1;
SET @bonus_section_id = (SELECT id FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @bonus_formation_id, @bonus_section_id, 67 WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @bonus_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @bonus_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @bonus_section_id, noteMinimale = 67 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Une tranche est très noire et large. Quelle piste est logique ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Une tranche est très noire et large. Quelle piste est logique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pourquoi préférer plusieurs tests courts à un grand essai ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Pourquoi préférer plusieurs tests courts à un grand essai ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Une gravure est dédoublée. Quelles causes vérifier ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Une gravure est dédoublée. Quelles causes vérifier ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Énergie trop élevée ou vitesse trop faible', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Écran trop sombre', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Fichier enregistré sur le mauvais disque', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nom de matière trop long', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Limiter le gaspillage et comparer les réglages en sécurité', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Éviter de mesurer l’épaisseur', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Supprimer le besoin d’extraction', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pouvoir quitter la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Jeu ou déplacement de la matière', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Doublons dans le fichier', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couleur du capot', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nombre de comptes utilisateurs', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;

-- ============================================================
-- Formation soudure électronique
-- ============================================================
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation soudure électronique' AND (categorie IS NULL OR categorie NOT IN ('Quiz interne', 'Validation physique')) ORDER BY id LIMIT 1);

-- Section 1 : Installer un poste sûr et ventilé
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Installer un poste sûr et ventilé', '{"intro":"Un poste de soudure bien organisé limite les brûlures, les fumées et les erreurs de montage avant même le premier point de soudure.","objectives":["Installer extraction et support","Repérer les zones chaudes","Organiser composants et outils"],"steps":[{"title":"Dégager le poste","text":"Éloignez papiers, câbles libres et liquides de la panne et du support."},{"title":"Activer l’extraction","text":"Placez l’aspiration près de la zone de travail sans gêner les gestes."},{"title":"Préparer les outils","text":"Support stable, éponge ou laine adaptée, pince, lunettes et composants identifiés."}],"callouts":[{"type":"warning","title":"Panne chaude","text":"Reposez toujours le fer dans son support, jamais sur la table."},{"type":"tip","title":"Organisation","text":"Travaillez du composant le plus bas vers le plus haut pour garder la carte stable."}]}', NULL, 1, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 1);
UPDATE SECTION SET titre = 'Installer un poste sûr et ventilé', contenu = '{"intro":"Un poste de soudure bien organisé limite les brûlures, les fumées et les erreurs de montage avant même le premier point de soudure.","objectives":["Installer extraction et support","Repérer les zones chaudes","Organiser composants et outils"],"steps":[{"title":"Dégager le poste","text":"Éloignez papiers, câbles libres et liquides de la panne et du support."},{"title":"Activer l’extraction","text":"Placez l’aspiration près de la zone de travail sans gêner les gestes."},{"title":"Préparer les outils","text":"Support stable, éponge ou laine adaptée, pince, lunettes et composants identifiés."}],"callouts":[{"type":"warning","title":"Panne chaude","text":"Reposez toujours le fer dans son support, jamais sur la table."},{"type":"tip","title":"Organisation","text":"Travaillez du composant le plus bas vers le plus haut pour garder la carte stable."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 1;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 1 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Station de soudure · 1';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Station de soudure.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Station de soudure.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Installer un poste sûr et ventilé', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Installer un poste sûr et ventilé', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Où doit être posé le fer entre deux soudures ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Où doit être posé le fer entre deux soudures ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quels éléments rendent le poste plus sûr ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quels éléments rendent le poste plus sûr ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dans son support stable prévu à cet effet', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Sur le bord de la table', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Sur le câble d’alimentation', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dans une boîte en plastique', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Extraction des fumées active', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Zone dégagée et support stable', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Papier sous la panne', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Câbles emmêlés', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 2 : Réaliser une soudure propre et fiable
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Réaliser une soudure propre et fiable', '{"intro":"Une bonne soudure chauffe simultanément la pastille et la patte, puis apporte juste assez d’étain pour former un joint brillant et régulier.","objectives":["Chauffer les deux surfaces","Doser l’étain","Reconnaître un joint correct"],"steps":[{"title":"Nettoyer et étamer","text":"Gardez la panne propre et légèrement étamée pour transmettre correctement la chaleur."},{"title":"Chauffer la jonction","text":"Posez la panne au contact de la pastille et de la patte avant d’apporter l’étain."},{"title":"Retirer dans le bon ordre","text":"Retirez d’abord le fil d’étain, puis la panne, sans bouger le composant pendant le refroidissement."}],"callouts":[{"type":"check","title":"Aspect recherché","text":"Joint concave, mouillage visible, sans boule ni pont avec la piste voisine."},{"type":"warning","title":"Surchauffe","text":"Une chauffe trop longue peut décoller une pastille ou endommager le composant."}]}', NULL, 2, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);
UPDATE SECTION SET titre = 'Réaliser une soudure propre et fiable', contenu = '{"intro":"Une bonne soudure chauffe simultanément la pastille et la patte, puis apporte juste assez d’étain pour former un joint brillant et régulier.","objectives":["Chauffer les deux surfaces","Doser l’étain","Reconnaître un joint correct"],"steps":[{"title":"Nettoyer et étamer","text":"Gardez la panne propre et légèrement étamée pour transmettre correctement la chaleur."},{"title":"Chauffer la jonction","text":"Posez la panne au contact de la pastille et de la patte avant d’apporter l’étain."},{"title":"Retirer dans le bon ordre","text":"Retirez d’abord le fil d’étain, puis la panne, sans bouger le composant pendant le refroidissement."}],"callouts":[{"type":"check","title":"Aspect recherché","text":"Joint concave, mouillage visible, sans boule ni pont avec la piste voisine."},{"type":"warning","title":"Surchauffe","text":"Une chauffe trop longue peut décoller une pastille ou endommager le composant."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 2;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 2 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Station de soudure · 2';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Station de soudure.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Station de soudure.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Réaliser une soudure propre et fiable', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Réaliser une soudure propre et fiable', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Où faut-il apporter l’étain ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Où faut-il apporter l’étain ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quels signes correspondent à une soudure correcte ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quels signes correspondent à une soudure correcte ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Sur la jonction chauffée entre la pastille et la patte', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement sur la pointe du fer', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Sur le masque de soudure', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Sur le câble secteur', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Mouillage de la pastille et de la patte', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Forme régulière sans pont', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Grosse boule isolée', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Composant mobile pendant le refroidissement', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 3 : Respecter polarités et composants sensibles
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Respecter polarités et composants sensibles', '{"intro":"Avant de souder, vérifiez la référence, la polarité et le sens de montage. Une diode, une LED ou un condensateur électrolytique inversé peut empêcher le circuit de fonctionner.","objectives":["Lire un repère de polarité","Éviter les contraintes mécaniques","Protéger les composants sensibles"],"steps":[{"title":"Lire le plan","text":"Comparez le schéma, la sérigraphie et le composant avant insertion."},{"title":"Identifier les repères","text":"Bague de diode, méplat de LED, signe négatif du condensateur et détrompeur de circuit intégré."},{"title":"Fixer sans forcer","text":"Pliez les pattes avec un rayon raisonnable et évitez de chauffer inutilement le boîtier."}],"callouts":[{"type":"tip","title":"Avant de couper","text":"Contrôlez une dernière fois l’orientation avant de raccourcir les pattes."},{"type":"warning","title":"ESD","text":"Manipulez les composants sensibles à l’électricité statique selon la procédure de l’atelier."}]}', NULL, 3, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 3);
UPDATE SECTION SET titre = 'Respecter polarités et composants sensibles', contenu = '{"intro":"Avant de souder, vérifiez la référence, la polarité et le sens de montage. Une diode, une LED ou un condensateur électrolytique inversé peut empêcher le circuit de fonctionner.","objectives":["Lire un repère de polarité","Éviter les contraintes mécaniques","Protéger les composants sensibles"],"steps":[{"title":"Lire le plan","text":"Comparez le schéma, la sérigraphie et le composant avant insertion."},{"title":"Identifier les repères","text":"Bague de diode, méplat de LED, signe négatif du condensateur et détrompeur de circuit intégré."},{"title":"Fixer sans forcer","text":"Pliez les pattes avec un rayon raisonnable et évitez de chauffer inutilement le boîtier."}],"callouts":[{"type":"tip","title":"Avant de couper","text":"Contrôlez une dernière fois l’orientation avant de raccourcir les pattes."},{"type":"warning","title":"ESD","text":"Manipulez les composants sensibles à l’électricité statique selon la procédure de l’atelier."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 3;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 3 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Station de soudure · 3';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Station de soudure.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Station de soudure.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Respecter polarités et composants sensibles', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Respecter polarités et composants sensibles', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quand faut-il vérifier la polarité d’un composant ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Quand faut-il vérifier la polarité d’un composant ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quels composants sont typiquement polarisés ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quels composants sont typiquement polarisés ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Avant de le souder et avant de couper ses pattes', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement après mise sous tension', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Après avoir retiré la sérigraphie', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Seulement si le circuit chauffe', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'LED', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Condensateur électrolytique', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Résistance standard', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Fil de cuivre nu', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 4 : Contrôler, corriger et ranger le poste
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Contrôler, corriger et ranger le poste', '{"intro":"Le contrôle visuel et électrique révèle les ponts, soudures froides et inversions avant la mise sous tension. La retouche doit rester courte et maîtrisée.","objectives":["Inspecter les joints","Dessouder sans arracher","Laisser le poste propre"],"steps":[{"title":"Inspecter à la loupe","text":"Cherchez ponts, manque d’étain, boule, fissure et projection métallique."},{"title":"Tester hors tension","text":"Utilisez le multimètre pour contrôler continuité et absence de court-circuit selon le montage."},{"title":"Nettoyer et arrêter","text":"Éteignez le fer, laissez-le refroidir, rangez les outils et éliminez les chutes dans le contenant prévu."}],"callouts":[{"type":"warning","title":"Dessoudage","text":"Ne tirez jamais sur une patte tant que l’étain n’est pas complètement fondu."},{"type":"check","title":"Avant alimentation","text":"Polarités, courts-circuits, valeur des composants et sens des connecteurs contrôlés."}]}', NULL, 4, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 4);
UPDATE SECTION SET titre = 'Contrôler, corriger et ranger le poste', contenu = '{"intro":"Le contrôle visuel et électrique révèle les ponts, soudures froides et inversions avant la mise sous tension. La retouche doit rester courte et maîtrisée.","objectives":["Inspecter les joints","Dessouder sans arracher","Laisser le poste propre"],"steps":[{"title":"Inspecter à la loupe","text":"Cherchez ponts, manque d’étain, boule, fissure et projection métallique."},{"title":"Tester hors tension","text":"Utilisez le multimètre pour contrôler continuité et absence de court-circuit selon le montage."},{"title":"Nettoyer et arrêter","text":"Éteignez le fer, laissez-le refroidir, rangez les outils et éliminez les chutes dans le contenant prévu."}],"callouts":[{"type":"warning","title":"Dessoudage","text":"Ne tirez jamais sur une patte tant que l’étain n’est pas complètement fondu."},{"type":"check","title":"Avant alimentation","text":"Polarités, courts-circuits, valeur des composants et sens des connecteurs contrôlés."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 4;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 4 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Station de soudure · 4';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Station de soudure.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Station de soudure.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Contrôler, corriger et ranger le poste', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Contrôler, corriger et ranger le poste', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Que faut-il vérifier avant la première mise sous tension ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Que faut-il vérifier avant la première mise sous tension ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Comment retirer un composant sans arracher la pastille ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Comment retirer un composant sans arracher la pastille ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'L’absence de court-circuit et les polarités', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du tapis', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nombre de fenêtres ouvertes', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La vitesse du ventilateur de salle', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Faire fondre correctement l’étain et utiliser l’outil de dessoudage adapté', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Tirer très fort à froid', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Gratter la piste avec un tournevis', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Chauffer le boîtier au briquet', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Quiz bonus : Défi expert · diagnostiquer une carte après soudure
SET @bonus_title = '[FABOS BONUS] Station de soudure · Défi expert';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @bonus_title, 'Quiz bonus facultatif pour Station de soudure. Il ne compte pas dans la progression.', parent.image, 'Quiz interne', parent.niveau, '4 min', 'FabOS', NULL, 'Approfondir le diagnostic machine.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;'), 'Quiz bonus'
FROM FORMATION parent WHERE parent.id = @parent_id AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne');
SET @bonus_formation_id = (SELECT id FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Quiz bonus facultatif pour Station de soudure. Il ne compte pas dans la progression.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;') WHERE id = @bonus_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @bonus_formation_id, 'Défi expert · diagnostiquer une carte après soudure', 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.', NULL, 1, CURRENT_TIMESTAMP
WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Défi expert · diagnostiquer une carte après soudure', contenu = 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.' WHERE formationId = @bonus_formation_id AND ordre = 1;
SET @bonus_section_id = (SELECT id FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @bonus_formation_id, @bonus_section_id, 67 WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @bonus_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @bonus_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @bonus_section_id, noteMinimale = 67 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Une soudure est mate et granuleuse. Quelle cause est probable ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Une soudure est mate et granuleuse. Quelle cause est probable ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Le multimètre indique un court-circuit entre deux rails. Que faire d’abord ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Le multimètre indique un court-circuit entre deux rails. Que faire d’abord ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles méthodes limitent l’endommagement lors d’une retouche ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Quelles méthodes limitent l’endommagement lors d’une retouche ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Mauvais mouillage ou mouvement pendant le refroidissement', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Trop de lumière', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Carte trop récente', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Valeur de résistance correcte', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Inspecter les ponts et l’orientation des composants hors tension', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Alimenter plus fort', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer si la carte est petite', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Changer de câble USB uniquement', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Temps de chauffe court et flux adapté', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Outil de dessoudage approprié', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Tirer à froid', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Gratter la pastille', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;

-- ============================================================
-- Formation découpe vinyle
-- ============================================================
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation découpe vinyle' AND (categorie IS NULL OR categorie NOT IN ('Quiz interne', 'Validation physique')) ORDER BY id LIMIT 1);

-- Section 1 : Choisir le support et préparer le visuel
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Choisir le support et préparer le visuel', '{"intro":"La découpe vinyle fonctionne avec des tracés vectoriels simples. Le choix du film et la taille des détails déterminent directement la facilité d’échenillage.","objectives":["Choisir vinyle ou flex","Préparer un tracé vectoriel","Adapter les détails à la lame"],"steps":[{"title":"Définir l’usage","text":"Vinyle adhésif pour surface rigide, flex textile pour transfert à chaud, avec le type validé par l’atelier."},{"title":"Nettoyer le fichier","text":"Convertissez les textes en courbes, fermez les tracés et retirez les doublons."},{"title":"Simplifier les détails","text":"Épaississez les éléments trop fins et laissez assez d’espace pour l’échenillage."}],"callouts":[{"type":"tip","title":"Petit format","text":"Un détail lisible à l’écran peut être trop fin pour rester collé après découpe."},{"type":"check","title":"Fichier prêt","text":"Échelle réelle, contours uniques et miroir activé seulement pour le flex textile."}]}', NULL, 1, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 1);
UPDATE SECTION SET titre = 'Choisir le support et préparer le visuel', contenu = '{"intro":"La découpe vinyle fonctionne avec des tracés vectoriels simples. Le choix du film et la taille des détails déterminent directement la facilité d’échenillage.","objectives":["Choisir vinyle ou flex","Préparer un tracé vectoriel","Adapter les détails à la lame"],"steps":[{"title":"Définir l’usage","text":"Vinyle adhésif pour surface rigide, flex textile pour transfert à chaud, avec le type validé par l’atelier."},{"title":"Nettoyer le fichier","text":"Convertissez les textes en courbes, fermez les tracés et retirez les doublons."},{"title":"Simplifier les détails","text":"Épaississez les éléments trop fins et laissez assez d’espace pour l’échenillage."}],"callouts":[{"type":"tip","title":"Petit format","text":"Un détail lisible à l’écran peut être trop fin pour rester collé après découpe."},{"type":"check","title":"Fichier prêt","text":"Échelle réelle, contours uniques et miroir activé seulement pour le flex textile."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 1;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 1 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Découpeuse vinyle · 1';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Découpeuse vinyle.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Découpeuse vinyle.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Choisir le support et préparer le visuel', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Choisir le support et préparer le visuel', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quand faut-il généralement mettre le motif en miroir ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Quand faut-il généralement mettre le motif en miroir ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quels contrôles concernent le fichier vectoriel ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quels contrôles concernent le fichier vectoriel ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour un flex textile appliqué par transfert', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour tout vinyle adhésif', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour nettoyer la lame', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour mesurer le rouleau', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Supprimer les tracés doublés', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Convertir les textes si nécessaire', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ajouter des images bitmap sans contour', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Changer la couleur du bureau', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 2 : Charger le rouleau et régler la lame
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Charger le rouleau et régler la lame', '{"intro":"Un rouleau droit et des galets bien placés évitent les décalages. La lame doit sortir juste assez pour couper le film sans traverser son support.","objectives":["Aligner le support","Positionner les galets","Régler la sortie de lame"],"steps":[{"title":"Aligner la matière","text":"Présentez le bord droit, répartissez la tension et vérifiez que la largeur utile est suffisante."},{"title":"Placer les galets","text":"Positionnez-les dans les zones autorisées et sur la matière, pas au bord du vide."},{"title":"Contrôler la lame","text":"La pointe doit à peine dépasser ; une sortie excessive abîme le support et les angles."}],"callouts":[{"type":"warning","title":"Lame fragile","text":"Ne touchez pas la pointe et ne la laissez pas tomber sur le plateau."},{"type":"info","title":"Largeur utile","text":"La machine mesure la zone réellement disponible entre les galets."}]}', NULL, 2, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);
UPDATE SECTION SET titre = 'Charger le rouleau et régler la lame', contenu = '{"intro":"Un rouleau droit et des galets bien placés évitent les décalages. La lame doit sortir juste assez pour couper le film sans traverser son support.","objectives":["Aligner le support","Positionner les galets","Régler la sortie de lame"],"steps":[{"title":"Aligner la matière","text":"Présentez le bord droit, répartissez la tension et vérifiez que la largeur utile est suffisante."},{"title":"Placer les galets","text":"Positionnez-les dans les zones autorisées et sur la matière, pas au bord du vide."},{"title":"Contrôler la lame","text":"La pointe doit à peine dépasser ; une sortie excessive abîme le support et les angles."}],"callouts":[{"type":"warning","title":"Lame fragile","text":"Ne touchez pas la pointe et ne la laissez pas tomber sur le plateau."},{"type":"info","title":"Largeur utile","text":"La machine mesure la zone réellement disponible entre les galets."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 2;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 2 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Découpeuse vinyle · 2';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Découpeuse vinyle.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Découpeuse vinyle.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Charger le rouleau et régler la lame', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Charger le rouleau et régler la lame', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Comment doit sortir la lame du porte-lame ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Comment doit sortir la lame du porte-lame ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Où placer les galets d’entraînement ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Où placer les galets d’entraînement ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Juste assez pour traverser le film, pas le support', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le plus possible', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pas du tout', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Jusqu’à toucher le galet', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Sur la matière et dans les zones autorisées', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Hors du rouleau', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Tous du même côté', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Sur un bord déchiré', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 3 : Faire un test de coupe et lancer le travail
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Faire un test de coupe et lancer le travail', '{"intro":"Le petit test de coupe est indispensable : le motif doit se détacher proprement tandis que le papier support reste presque intact.","objectives":["Lire un test de coupe","Ajuster force et vitesse","Vérifier l’origine"],"steps":[{"title":"Définir l’origine","text":"Placez la tête dans une zone libre et enregistrez l’origine avant l’envoi."},{"title":"Lancer le test","text":"Échenillez le petit motif et observez si le film est coupé sans marquer profondément le liner."},{"title":"Ajuster progressivement","text":"Modifiez la force par petits pas avant de changer la sortie de lame."}],"callouts":[{"type":"tip","title":"Ordre de réglage","text":"Commencez par la force ; la lame ne doit être davantage sortie qu’en dernier recours."},{"type":"warning","title":"Sans test","text":"Un mauvais réglage peut ruiner tout le rouleau ou endommager la bande de coupe."}]}', NULL, 3, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 3);
UPDATE SECTION SET titre = 'Faire un test de coupe et lancer le travail', contenu = '{"intro":"Le petit test de coupe est indispensable : le motif doit se détacher proprement tandis que le papier support reste presque intact.","objectives":["Lire un test de coupe","Ajuster force et vitesse","Vérifier l’origine"],"steps":[{"title":"Définir l’origine","text":"Placez la tête dans une zone libre et enregistrez l’origine avant l’envoi."},{"title":"Lancer le test","text":"Échenillez le petit motif et observez si le film est coupé sans marquer profondément le liner."},{"title":"Ajuster progressivement","text":"Modifiez la force par petits pas avant de changer la sortie de lame."}],"callouts":[{"type":"tip","title":"Ordre de réglage","text":"Commencez par la force ; la lame ne doit être davantage sortie qu’en dernier recours."},{"type":"warning","title":"Sans test","text":"Un mauvais réglage peut ruiner tout le rouleau ou endommager la bande de coupe."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 3;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 3 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Découpeuse vinyle · 3';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Découpeuse vinyle.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Découpeuse vinyle.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Faire un test de coupe et lancer le travail', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Faire un test de coupe et lancer le travail', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Que doit montrer un bon test de coupe ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Que doit montrer un bon test de coupe ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel réglage ajuster en premier si le film n’est pas assez coupé ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quel réglage ajuster en premier si le film n’est pas assez coupé ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le film se retire proprement et le support reste peu marqué', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le support est entièrement traversé', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le motif reste attaché partout', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le rouleau se décale', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La force, par petites étapes', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La sortie de lame au maximum', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du motif', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La largeur du rouleau', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 4 : Écheniller, transférer et finir proprement
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Écheniller, transférer et finir proprement', '{"intro":"Après découpe, l’échenillage retire les zones inutiles. Le film de transfert maintient les éléments alignés jusqu’à la pose sur une surface propre.","objectives":["Écheniller sans perdre les détails","Poser un transfert sans bulles","Ranger les chutes"],"steps":[{"title":"Écheniller méthodiquement","text":"Commencez par les grandes zones puis utilisez une pince pour les petits intérieurs."},{"title":"Appliquer le transfert","text":"Marouflez du centre vers l’extérieur et vérifiez que tous les éléments adhèrent au film."},{"title":"Poser et nettoyer","text":"Dégraissez la surface, posez progressivement puis retirez le transfert à angle faible."}],"callouts":[{"type":"tip","title":"Détails fragiles","text":"Gardez le motif à plat et évitez de tirer rapidement sur les petites formes."},{"type":"check","title":"Fin de session","text":"Chutes triées, lame protégée, rouleau identifié et poste nettoyé."}]}', NULL, 4, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 4);
UPDATE SECTION SET titre = 'Écheniller, transférer et finir proprement', contenu = '{"intro":"Après découpe, l’échenillage retire les zones inutiles. Le film de transfert maintient les éléments alignés jusqu’à la pose sur une surface propre.","objectives":["Écheniller sans perdre les détails","Poser un transfert sans bulles","Ranger les chutes"],"steps":[{"title":"Écheniller méthodiquement","text":"Commencez par les grandes zones puis utilisez une pince pour les petits intérieurs."},{"title":"Appliquer le transfert","text":"Marouflez du centre vers l’extérieur et vérifiez que tous les éléments adhèrent au film."},{"title":"Poser et nettoyer","text":"Dégraissez la surface, posez progressivement puis retirez le transfert à angle faible."}],"callouts":[{"type":"tip","title":"Détails fragiles","text":"Gardez le motif à plat et évitez de tirer rapidement sur les petites formes."},{"type":"check","title":"Fin de session","text":"Chutes triées, lame protégée, rouleau identifié et poste nettoyé."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 4;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 4 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Découpeuse vinyle · 4';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Découpeuse vinyle.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Découpeuse vinyle.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Écheniller, transférer et finir proprement', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Écheniller, transférer et finir proprement', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pourquoi utiliser un film de transfert ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pourquoi utiliser un film de transfert ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions améliorent la pose ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelles actions améliorent la pose ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Maintenir les éléments du motif alignés pendant la pose', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Augmenter la force de coupe', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nettoyer la lame', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Mesurer le rouleau', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nettoyer et dégraisser la surface', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Maroufler progressivement', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Plier fortement le motif', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Poser sur une surface poussiéreuse', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Quiz bonus : Défi expert · résoudre les défauts de découpe vinyle
SET @bonus_title = '[FABOS BONUS] Découpeuse vinyle · Défi expert';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @bonus_title, 'Quiz bonus facultatif pour Découpeuse vinyle. Il ne compte pas dans la progression.', parent.image, 'Quiz interne', parent.niveau, '4 min', 'FabOS', NULL, 'Approfondir le diagnostic machine.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;'), 'Quiz bonus'
FROM FORMATION parent WHERE parent.id = @parent_id AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne');
SET @bonus_formation_id = (SELECT id FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Quiz bonus facultatif pour Découpeuse vinyle. Il ne compte pas dans la progression.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;') WHERE id = @bonus_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @bonus_formation_id, 'Défi expert · résoudre les défauts de découpe vinyle', 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.', NULL, 1, CURRENT_TIMESTAMP
WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Défi expert · résoudre les défauts de découpe vinyle', contenu = 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.' WHERE formationId = @bonus_formation_id AND ordre = 1;
SET @bonus_section_id = (SELECT id FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @bonus_formation_id, @bonus_section_id, 67 WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @bonus_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @bonus_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @bonus_section_id, noteMinimale = 67 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Les angles du motif se soulèvent pendant la coupe. Quelle piste vérifier ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Les angles du motif se soulèvent pendant la coupe. Quelle piste vérifier ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Le rouleau dérive sur une grande longueur. Quelles causes sont plausibles ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Le rouleau dérive sur une grande longueur. Quelles causes sont plausibles ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Le liner est profondément marqué. Quel ajustement est logique ?', 'single', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Le liner est profondément marqué. Quel ajustement est logique ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 3;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Lame trop sortie, force excessive ou détail trop fin', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nom de fichier trop long', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Température de l’écran', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Connexion au calendrier', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Mauvais alignement initial', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Galet mal positionné', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couleur du vinyle', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Police convertie en courbes', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Réduire la force et vérifier la sortie de lame', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Augmenter la vitesse au maximum', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ajouter un second passage', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Retourner les galets', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;

-- ============================================================
-- Formation fraiseuse CNC
-- ============================================================
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation fraiseuse CNC' AND (categorie IS NULL OR categorie NOT IN ('Quiz interne', 'Validation physique')) ORDER BY id LIMIT 1);

-- Section 1 : Sécuriser la zone et brider la pièce
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Sécuriser la zone et brider la pièce', '{"intro":"Sur une CNC, une pièce mal bridée ou un outil exposé transforme rapidement un petit défaut en danger. Le montage doit être rigide, accessible et compatible avec le parcours d’outil.","objectives":["Choisir un bridage rigide","Éviter les collisions","Repérer arrêt et protections"],"steps":[{"title":"Préparer la table","text":"Nettoyez la surface, placez le martyr et vérifiez qu’aucune vis ne se trouve dans le parcours prévu."},{"title":"Brider la pièce","text":"Bloquez les déplacements et soulèvements sans déformer la matière."},{"title":"Tester l’accessibilité","text":"Vérifiez que la broche, la pince et les brides ne peuvent pas se rencontrer."}],"callouts":[{"type":"warning","title":"Pièce libre","text":"N’usinez jamais une pièce tenue à la main ou simplement posée sur la table."},{"type":"check","title":"Avant mise en route","text":"Protections en place, arrêt accessible, aspiration prête et zone évacuée."}]}', NULL, 1, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 1);
UPDATE SECTION SET titre = 'Sécuriser la zone et brider la pièce', contenu = '{"intro":"Sur une CNC, une pièce mal bridée ou un outil exposé transforme rapidement un petit défaut en danger. Le montage doit être rigide, accessible et compatible avec le parcours d’outil.","objectives":["Choisir un bridage rigide","Éviter les collisions","Repérer arrêt et protections"],"steps":[{"title":"Préparer la table","text":"Nettoyez la surface, placez le martyr et vérifiez qu’aucune vis ne se trouve dans le parcours prévu."},{"title":"Brider la pièce","text":"Bloquez les déplacements et soulèvements sans déformer la matière."},{"title":"Tester l’accessibilité","text":"Vérifiez que la broche, la pince et les brides ne peuvent pas se rencontrer."}],"callouts":[{"type":"warning","title":"Pièce libre","text":"N’usinez jamais une pièce tenue à la main ou simplement posée sur la table."},{"type":"check","title":"Avant mise en route","text":"Protections en place, arrêt accessible, aspiration prête et zone évacuée."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 1;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 1 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Fraiseuse CNC · 1';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Fraiseuse CNC.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Fraiseuse CNC.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Sécuriser la zone et brider la pièce', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Sécuriser la zone et brider la pièce', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pourquoi faut-il vérifier l’emplacement des vis et brides ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pourquoi faut-il vérifier l’emplacement des vis et brides ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quels objectifs doit remplir le bridage ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quels objectifs doit remplir le bridage ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour éviter une collision avec l’outil ou la broche', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour modifier la couleur du fichier', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour accélérer le réseau', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour éteindre l’aspiration', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Empêcher la pièce de glisser', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Empêcher la pièce de se soulever', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser la pièce vibrer', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bloquer l’arrêt d’urgence', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 2 : Choisir l’outil et préparer le parcours CAM
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Choisir l’outil et préparer le parcours CAM', '{"intro":"Le parcours CAM doit correspondre à la matière, à la fraise et à la machine. Profondeur par passe, avance et vitesse de rotation sont liées.","objectives":["Associer fraise et opération","Définir passes et attaches","Simuler les collisions"],"steps":[{"title":"Choisir la fraise","text":"Diamètre, longueur utile, nombre de dents et géométrie doivent convenir à la matière et au détail."},{"title":"Configurer les opérations","text":"Définissez dégrossissage, finition, poches, contours et attaches dans un ordre logique."},{"title":"Simuler","text":"Contrôlez profondeur maximale, stock restant, mouvements rapides et zones de bridage."}],"callouts":[{"type":"tip","title":"Attaches","text":"Les tabs maintiennent les pièces découpées jusqu’à la fin du contour."},{"type":"warning","title":"Longueur sortie","text":"Une fraise trop sortie vibre davantage et casse plus facilement."}]}', NULL, 2, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);
UPDATE SECTION SET titre = 'Choisir l’outil et préparer le parcours CAM', contenu = '{"intro":"Le parcours CAM doit correspondre à la matière, à la fraise et à la machine. Profondeur par passe, avance et vitesse de rotation sont liées.","objectives":["Associer fraise et opération","Définir passes et attaches","Simuler les collisions"],"steps":[{"title":"Choisir la fraise","text":"Diamètre, longueur utile, nombre de dents et géométrie doivent convenir à la matière et au détail."},{"title":"Configurer les opérations","text":"Définissez dégrossissage, finition, poches, contours et attaches dans un ordre logique."},{"title":"Simuler","text":"Contrôlez profondeur maximale, stock restant, mouvements rapides et zones de bridage."}],"callouts":[{"type":"tip","title":"Attaches","text":"Les tabs maintiennent les pièces découpées jusqu’à la fin du contour."},{"type":"warning","title":"Longueur sortie","text":"Une fraise trop sortie vibre davantage et casse plus facilement."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 2;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 2 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Fraiseuse CNC · 2';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Fraiseuse CNC.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Fraiseuse CNC.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Choisir l’outil et préparer le parcours CAM', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Choisir l’outil et préparer le parcours CAM', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'À quoi servent les attaches sur un contour traversant ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'À quoi servent les attaches sur un contour traversant ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Que faut-il vérifier dans la simulation CAM ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Que faut-il vérifier dans la simulation CAM ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Maintenir la pièce jusqu’à la fin de l’usinage', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Augmenter la vitesse réseau', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Remplacer le bridage principal', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Refroidir la broche', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Les profondeurs et mouvements rapides', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Les zones de bridage', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le thème clair ou sombre', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nombre d’utilisateurs connectés', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 3 : Régler les origines et faire un essai à vide
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Régler les origines et faire un essai à vide', '{"intro":"Les origines relient le fichier numérique à la pièce réelle. Une erreur de zéro peut envoyer l’outil dans la table ou hors matière.","objectives":["Définir X, Y et Z","Vérifier le sens des axes","Faire un dry-run"],"steps":[{"title":"Monter l’outil","text":"Nettoyez la pince, insérez suffisamment la fraise et serrez avec les outils prévus."},{"title":"Prendre les origines","text":"Réglez X/Y selon le repère CAM et Z selon la surface ou le martyr choisi."},{"title":"Tester sans couper","text":"Relevez Z ou utilisez le mode prévu pour parcourir les limites à vitesse réduite."}],"callouts":[{"type":"warning","title":"Z incorrect","text":"Un zéro Z pris au mauvais niveau peut casser l’outil ou usiner la table."},{"type":"info","title":"Dry-run","text":"L’essai à vide vérifie l’enveloppe et les mouvements sans charge de coupe."}]}', NULL, 3, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 3);
UPDATE SECTION SET titre = 'Régler les origines et faire un essai à vide', contenu = '{"intro":"Les origines relient le fichier numérique à la pièce réelle. Une erreur de zéro peut envoyer l’outil dans la table ou hors matière.","objectives":["Définir X, Y et Z","Vérifier le sens des axes","Faire un dry-run"],"steps":[{"title":"Monter l’outil","text":"Nettoyez la pince, insérez suffisamment la fraise et serrez avec les outils prévus."},{"title":"Prendre les origines","text":"Réglez X/Y selon le repère CAM et Z selon la surface ou le martyr choisi."},{"title":"Tester sans couper","text":"Relevez Z ou utilisez le mode prévu pour parcourir les limites à vitesse réduite."}],"callouts":[{"type":"warning","title":"Z incorrect","text":"Un zéro Z pris au mauvais niveau peut casser l’outil ou usiner la table."},{"type":"info","title":"Dry-run","text":"L’essai à vide vérifie l’enveloppe et les mouvements sans charge de coupe."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 3;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 3 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Fraiseuse CNC · 3';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Fraiseuse CNC.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Fraiseuse CNC.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Régler les origines et faire un essai à vide', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Régler les origines et faire un essai à vide', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pourquoi faire un essai à vide ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pourquoi faire un essai à vide ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelle erreur peut provoquer un mauvais zéro Z ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelle erreur peut provoquer un mauvais zéro Z ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier le parcours et l’enveloppe avant de couper', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Affûter automatiquement la fraise', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Supprimer le besoin de bridage', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Changer la matière', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Une profondeur incorrecte ou une collision avec la table', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Une couleur différente', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Un fichier plus petit', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Une meilleure aspiration', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 4 : Lancer, surveiller et gérer les incidents
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Lancer, surveiller et gérer les incidents', '{"intro":"Bruit, copeaux et vibration racontent ce qui se passe dans la coupe. Une surveillance active permet d’arrêter avant la casse ou le déplacement de la pièce.","objectives":["Observer la coupe","Identifier surcharge et vibration","Arrêter en sécurité"],"steps":[{"title":"Démarrer progressivement","text":"Lancez aspiration et broche selon la procédure, puis le programme en restant prêt à arrêter."},{"title":"Lire les signes","text":"Copeaux corrects, son stable et absence de vibration indiquent une coupe normale."},{"title":"Clore le travail","text":"Attendez l’arrêt complet, aspirez les copeaux, libérez la pièce et inspectez l’outil."}],"callouts":[{"type":"warning","title":"Ne jamais approcher","text":"N’essayez pas de retirer des copeaux à la main pendant la rotation."},{"type":"check","title":"Incident","text":"Arrêt, mise en sécurité, éloignement des personnes et signalement au responsable."}]}', NULL, 4, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 4);
UPDATE SECTION SET titre = 'Lancer, surveiller et gérer les incidents', contenu = '{"intro":"Bruit, copeaux et vibration racontent ce qui se passe dans la coupe. Une surveillance active permet d’arrêter avant la casse ou le déplacement de la pièce.","objectives":["Observer la coupe","Identifier surcharge et vibration","Arrêter en sécurité"],"steps":[{"title":"Démarrer progressivement","text":"Lancez aspiration et broche selon la procédure, puis le programme en restant prêt à arrêter."},{"title":"Lire les signes","text":"Copeaux corrects, son stable et absence de vibration indiquent une coupe normale."},{"title":"Clore le travail","text":"Attendez l’arrêt complet, aspirez les copeaux, libérez la pièce et inspectez l’outil."}],"callouts":[{"type":"warning","title":"Ne jamais approcher","text":"N’essayez pas de retirer des copeaux à la main pendant la rotation."},{"type":"check","title":"Incident","text":"Arrêt, mise en sécurité, éloignement des personnes et signalement au responsable."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 4;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 4 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Fraiseuse CNC · 4';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Fraiseuse CNC.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Fraiseuse CNC.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Lancer, surveiller et gérer les incidents', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Lancer, surveiller et gérer les incidents', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel signe peut indiquer une coupe en surcharge ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Quel signe peut indiquer une coupe en surcharge ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quand peut-on retirer les copeaux près de l’outil ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quand peut-on retirer les copeaux près de l’outil ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bruit qui augmente, vibration ou copeaux anormaux', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Barre de progression stable', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Écran allumé', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nom de fichier court', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Après l’arrêt complet avec l’outil adapté', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pendant la rotation avec les doigts', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pendant un mouvement rapide', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Avant d’arrêter la broche', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Quiz bonus : Défi expert · lire les symptômes d’un usinage
SET @bonus_title = '[FABOS BONUS] Fraiseuse CNC · Défi expert';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @bonus_title, 'Quiz bonus facultatif pour Fraiseuse CNC. Il ne compte pas dans la progression.', parent.image, 'Quiz interne', parent.niveau, '4 min', 'FabOS', NULL, 'Approfondir le diagnostic machine.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;'), 'Quiz bonus'
FROM FORMATION parent WHERE parent.id = @parent_id AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne');
SET @bonus_formation_id = (SELECT id FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Quiz bonus facultatif pour Fraiseuse CNC. Il ne compte pas dans la progression.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;') WHERE id = @bonus_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @bonus_formation_id, 'Défi expert · lire les symptômes d’un usinage', 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.', NULL, 1, CURRENT_TIMESTAMP
WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Défi expert · lire les symptômes d’un usinage', contenu = 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.' WHERE formationId = @bonus_formation_id AND ordre = 1;
SET @bonus_section_id = (SELECT id FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @bonus_formation_id, @bonus_section_id, 67 WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @bonus_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @bonus_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @bonus_section_id, noteMinimale = 67 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'La fraise vibre fortement dans une poche. Quelles pistes vérifier ?', 'multiple', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'La fraise vibre fortement dans une poche. Quelles pistes vérifier ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Les copeaux deviennent de la poussière très fine et l’outil chauffe. Quelle hypothèse est plausible ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Les copeaux deviennent de la poussière très fine et l’outil chauffe. Quelle hypothèse est plausible ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Une pièce bouge en fin de contour. Quelle cause est la plus directe ?', 'single', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Une pièce bouge en fin de contour. Quelle cause est la plus directe ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 3;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Longueur sortie et rigidité du montage', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Profondeur par passe et avance', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couleur du matériau', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Taille de l’écran', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Avance trop faible ou outil qui frotte', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Fichier trop récent', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Origine X trop proche', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Aspiration trop silencieuse', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bridage ou attaches insuffisants', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Broche trop propre', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Simulation trop longue', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nom de projet incorrect', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;

-- ============================================================
-- Formation oscilloscope numérique
-- ============================================================
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation oscilloscope numérique' AND (categorie IS NULL OR categorie NOT IN ('Quiz interne', 'Validation physique')) ORDER BY id LIMIT 1);

-- Section 1 : Brancher les masses sans créer de danger
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Brancher les masses sans créer de danger', '{"intro":"La pince de masse d’un oscilloscope de table est généralement reliée à la terre. La connecter au mauvais point peut créer un court-circuit immédiat.","objectives":["Identifier la référence de masse","Respecter les limites de tension","Choisir une sonde adaptée"],"steps":[{"title":"Comprendre la terre","text":"Vérifiez si la masse de la sonde est reliée au conducteur de protection de l’appareil."},{"title":"Repérer le point de référence","text":"Connectez la masse uniquement sur le zéro du circuit compatible avec la terre."},{"title":"Contrôler la tension","text":"Respectez la catégorie, la tension maximale et le facteur d’atténuation de la sonde."}],"callouts":[{"type":"warning","title":"Secteur","text":"Ne mesurez jamais directement un circuit secteur avec une sonde passive standard sans montage et isolation adaptés."},{"type":"check","title":"Avant connexion","text":"Référence identifiée, alimentation comprise et limites de la sonde vérifiées."}]}', NULL, 1, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 1);
UPDATE SECTION SET titre = 'Brancher les masses sans créer de danger', contenu = '{"intro":"La pince de masse d’un oscilloscope de table est généralement reliée à la terre. La connecter au mauvais point peut créer un court-circuit immédiat.","objectives":["Identifier la référence de masse","Respecter les limites de tension","Choisir une sonde adaptée"],"steps":[{"title":"Comprendre la terre","text":"Vérifiez si la masse de la sonde est reliée au conducteur de protection de l’appareil."},{"title":"Repérer le point de référence","text":"Connectez la masse uniquement sur le zéro du circuit compatible avec la terre."},{"title":"Contrôler la tension","text":"Respectez la catégorie, la tension maximale et le facteur d’atténuation de la sonde."}],"callouts":[{"type":"warning","title":"Secteur","text":"Ne mesurez jamais directement un circuit secteur avec une sonde passive standard sans montage et isolation adaptés."},{"type":"check","title":"Avant connexion","text":"Référence identifiée, alimentation comprise et limites de la sonde vérifiées."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 1;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 1 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Oscilloscope numérique · 1';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Oscilloscope numérique.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Oscilloscope numérique.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Brancher les masses sans créer de danger', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Brancher les masses sans créer de danger', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pourquoi la pince de masse peut-elle provoquer un court-circuit ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pourquoi la pince de masse peut-elle provoquer un court-circuit ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Que faut-il vérifier avant de connecter une sonde ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Que faut-il vérifier avant de connecter une sonde ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Elle est souvent reliée à la terre de protection', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Elle contient une batterie', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Elle change la fréquence', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Elle mesure uniquement le courant', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La référence de masse du circuit', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La tension maximale admissible', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La couleur du signal', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le nombre de fenêtres ouvertes', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 2 : Régler la sonde et le couplage d’entrée
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Régler la sonde et le couplage d’entrée', '{"intro":"Une sonde mal compensée déforme les fronts. Le réglage x1/x10 et le couplage AC/DC déterminent ce que l’oscilloscope affiche réellement.","objectives":["Compenser une sonde","Synchroniser x10 sur sonde et voie","Choisir AC ou DC"],"steps":[{"title":"Choisir l’atténuation","text":"Placez la sonde en x10 lorsque c’est adapté et configurez la voie avec le même facteur."},{"title":"Compenser","text":"Branchez le signal carré de calibration et ajustez pour obtenir des plateaux sans arrondi ni dépassement."},{"title":"Choisir le couplage","text":"DC conserve la composante continue ; AC la bloque pour observer une petite variation superposée."}],"callouts":[{"type":"tip","title":"Réglage x10","text":"Le x10 charge moins le circuit et offre souvent une meilleure bande passante."},{"type":"info","title":"Couplage AC","text":"Il ne signifie pas que le signal est alternatif : il retire seulement la composante continue."}]}', NULL, 2, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);
UPDATE SECTION SET titre = 'Régler la sonde et le couplage d’entrée', contenu = '{"intro":"Une sonde mal compensée déforme les fronts. Le réglage x1/x10 et le couplage AC/DC déterminent ce que l’oscilloscope affiche réellement.","objectives":["Compenser une sonde","Synchroniser x10 sur sonde et voie","Choisir AC ou DC"],"steps":[{"title":"Choisir l’atténuation","text":"Placez la sonde en x10 lorsque c’est adapté et configurez la voie avec le même facteur."},{"title":"Compenser","text":"Branchez le signal carré de calibration et ajustez pour obtenir des plateaux sans arrondi ni dépassement."},{"title":"Choisir le couplage","text":"DC conserve la composante continue ; AC la bloque pour observer une petite variation superposée."}],"callouts":[{"type":"tip","title":"Réglage x10","text":"Le x10 charge moins le circuit et offre souvent une meilleure bande passante."},{"type":"info","title":"Couplage AC","text":"Il ne signifie pas que le signal est alternatif : il retire seulement la composante continue."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 2;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 2 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Oscilloscope numérique · 2';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Oscilloscope numérique.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Oscilloscope numérique.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Régler la sonde et le couplage d’entrée', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Régler la sonde et le couplage d’entrée', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Que se passe-t-il si la sonde est en x10 mais la voie reste configurée en x1 ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Que se passe-t-il si la sonde est en x10 mais la voie reste configurée en x1 ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel couplage conserve la composante continue ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quel couplage conserve la composante continue ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La valeur affichée est fausse d’un facteur dix', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le signal disparaît toujours', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La fréquence double', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La masse devient flottante', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'DC', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'AC', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'GND uniquement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'FFT', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 3 : Stabiliser le signal avec base de temps et trigger
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Stabiliser le signal avec base de temps et trigger', '{"intro":"Un signal qui défile n’est pas forcément instable : il manque souvent un déclenchement cohérent. La base de temps et le niveau de trigger doivent être adaptés à la période.","objectives":["Afficher quelques périodes","Régler source et pente","Choisir un niveau de déclenchement"],"steps":[{"title":"Ajuster volts/div","text":"Utilisez l’écran sans écrêter le signal et gardez une marge autour des maxima."},{"title":"Ajuster temps/div","text":"Affichez une à plusieurs périodes pour lire la forme et la fréquence."},{"title":"Régler le trigger","text":"Choisissez la bonne voie, la pente utile et un niveau traversé de manière nette par le signal."}],"callouts":[{"type":"tip","title":"Signal stable","text":"Le trigger doit se produire au même événement à chaque acquisition."},{"type":"warning","title":"Écrêtage écran","text":"Un signal hors écran peut masquer une surtension ou fausser une mesure automatique."}]}', NULL, 3, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 3);
UPDATE SECTION SET titre = 'Stabiliser le signal avec base de temps et trigger', contenu = '{"intro":"Un signal qui défile n’est pas forcément instable : il manque souvent un déclenchement cohérent. La base de temps et le niveau de trigger doivent être adaptés à la période.","objectives":["Afficher quelques périodes","Régler source et pente","Choisir un niveau de déclenchement"],"steps":[{"title":"Ajuster volts/div","text":"Utilisez l’écran sans écrêter le signal et gardez une marge autour des maxima."},{"title":"Ajuster temps/div","text":"Affichez une à plusieurs périodes pour lire la forme et la fréquence."},{"title":"Régler le trigger","text":"Choisissez la bonne voie, la pente utile et un niveau traversé de manière nette par le signal."}],"callouts":[{"type":"tip","title":"Signal stable","text":"Le trigger doit se produire au même événement à chaque acquisition."},{"type":"warning","title":"Écrêtage écran","text":"Un signal hors écran peut masquer une surtension ou fausser une mesure automatique."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 3;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 3 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Oscilloscope numérique · 3';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Oscilloscope numérique.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Oscilloscope numérique.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Stabiliser le signal avec base de temps et trigger', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Stabiliser le signal avec base de temps et trigger', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quel réglage stabilise principalement un signal périodique qui défile ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Quel réglage stabilise principalement un signal périodique qui défile ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pour choisir le niveau de trigger, où faut-il le placer ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Pour choisir le niveau de trigger, où faut-il le placer ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le trigger', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La luminosité', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La langue de l’interface', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le port USB', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'À un niveau que le signal traverse clairement', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Toujours au maximum de l’écran', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Sous zéro dans tous les cas', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Sur une autre voie non connectée', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 4 : Mesurer et interpréter sans surinterpréter
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Mesurer et interpréter sans surinterpréter', '{"intro":"Les curseurs et mesures automatiques sont utiles, mais ils dépendent de l’échelle, du bruit, de la bande passante et de la qualité de la connexion.","objectives":["Mesurer amplitude et période","Reconnaître bruit et artefacts","Documenter les réglages"],"steps":[{"title":"Mesurer manuellement","text":"Utilisez divisions ou curseurs pour vérifier les mesures automatiques."},{"title":"Limiter le bruit","text":"Réduisez la boucle de masse, utilisez un ressort de masse et activez la limite de bande si approprié."},{"title":"Conserver une trace","text":"Notez échelles, atténuation, couplage et point de mesure avec la capture."}],"callouts":[{"type":"info","title":"Bande passante","text":"Une sonde ou un oscilloscope trop lent atténue les fronts rapides."},{"type":"check","title":"Mesure crédible","text":"Signal bien cadré, référence connue, sonde compensée et réglages documentés."}]}', NULL, 4, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 4);
UPDATE SECTION SET titre = 'Mesurer et interpréter sans surinterpréter', contenu = '{"intro":"Les curseurs et mesures automatiques sont utiles, mais ils dépendent de l’échelle, du bruit, de la bande passante et de la qualité de la connexion.","objectives":["Mesurer amplitude et période","Reconnaître bruit et artefacts","Documenter les réglages"],"steps":[{"title":"Mesurer manuellement","text":"Utilisez divisions ou curseurs pour vérifier les mesures automatiques."},{"title":"Limiter le bruit","text":"Réduisez la boucle de masse, utilisez un ressort de masse et activez la limite de bande si approprié."},{"title":"Conserver une trace","text":"Notez échelles, atténuation, couplage et point de mesure avec la capture."}],"callouts":[{"type":"info","title":"Bande passante","text":"Une sonde ou un oscilloscope trop lent atténue les fronts rapides."},{"type":"check","title":"Mesure crédible","text":"Signal bien cadré, référence connue, sonde compensée et réglages documentés."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 4;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 4 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Oscilloscope numérique · 4';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Oscilloscope numérique.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Oscilloscope numérique.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Mesurer et interpréter sans surinterpréter', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Mesurer et interpréter sans surinterpréter', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pourquoi comparer une mesure automatique avec les curseurs ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pourquoi comparer une mesure automatique avec les curseurs ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions peuvent réduire les artefacts de mesure ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelles actions peuvent réduire les artefacts de mesure ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour détecter une mauvaise détection ou un réglage inadapté', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour changer la couleur de la trace', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour désactiver la sonde', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pour isoler le secteur', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Raccourcir la connexion de masse', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier la compensation de sonde', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ajouter une longue boucle de masse', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ignorer la bande passante', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Quiz bonus : Défi expert · interpréter des signaux difficiles
SET @bonus_title = '[FABOS BONUS] Oscilloscope numérique · Défi expert';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @bonus_title, 'Quiz bonus facultatif pour Oscilloscope numérique. Il ne compte pas dans la progression.', parent.image, 'Quiz interne', parent.niveau, '4 min', 'FabOS', NULL, 'Approfondir le diagnostic machine.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;'), 'Quiz bonus'
FROM FORMATION parent WHERE parent.id = @parent_id AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne');
SET @bonus_formation_id = (SELECT id FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Quiz bonus facultatif pour Oscilloscope numérique. Il ne compte pas dans la progression.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;') WHERE id = @bonus_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @bonus_formation_id, 'Défi expert · interpréter des signaux difficiles', 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.', NULL, 1, CURRENT_TIMESTAMP
WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Défi expert · interpréter des signaux difficiles', contenu = 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.' WHERE formationId = @bonus_formation_id AND ordre = 1;
SET @bonus_section_id = (SELECT id FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @bonus_formation_id, @bonus_section_id, 67 WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @bonus_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @bonus_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @bonus_section_id, noteMinimale = 67 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Un front carré présente une oscillation qui disparaît avec un ressort de masse. Quelle cause est probable ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Un front carré présente une oscillation qui disparaît avec un ressort de masse. Quelle cause est probable ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Une petite ondulation sur 12 V est difficile à voir. Quel réglage est pertinent ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Une petite ondulation sur 12 V est difficile à voir. Quel réglage est pertinent ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Une mesure de fréquence saute entre plusieurs valeurs. Quelles pistes vérifier ?', 'multiple', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Une mesure de fréquence saute entre plusieurs valeurs. Quelles pistes vérifier ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 3;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Inductance de la longue pince de masse', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Fréquence du secteur trop basse', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Mémoire insuffisante', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couleur de trace', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couplage AC avec échelle verticale adaptée, en respectant les limites', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Passer la sonde en x1 sans vérifier', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Mettre la trace hors écran', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Désactiver le trigger', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Trigger instable', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Bruit ou amplitude insuffisante', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nom de la voie', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Taille du navigateur', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;

-- ============================================================
-- Formation brodeuse numérique
-- ============================================================
SET @parent_id = (SELECT id FROM FORMATION WHERE titre = 'Formation brodeuse numérique' AND (categorie IS NULL OR categorie NOT IN ('Quiz interne', 'Validation physique')) ORDER BY id LIMIT 1);

-- Section 1 : Choisir textile, stabilisateur et cadre
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Choisir textile, stabilisateur et cadre', '{"intro":"La qualité d’une broderie dépend du trio textile, stabilisateur et cadre. Un tissu mal maintenu se déforme, fronce ou décale les couleurs.","objectives":["Associer stabilisateur et textile","Choisir le bon cadre","Tendre sans déformer"],"steps":[{"title":"Analyser le textile","text":"Évaluez extensibilité, épaisseur, surface et sens du tissu avant de choisir le renfort."},{"title":"Choisir le stabilisateur","text":"Découpable, déchirable ou hydrosoluble selon la matière et la densité du motif."},{"title":"Mettre en cadre","text":"Tendez uniformément sans étirer le textile et vérifiez que la zone utile est libre."}],"callouts":[{"type":"tip","title":"Textile extensible","text":"Un stabilisateur adapté empêche le motif de se resserrer ou de gondoler."},{"type":"warning","title":"Cadre mal serré","text":"Un déplacement du tissu peut casser l’aiguille ou décaler toute la broderie."}]}', NULL, 1, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 1);
UPDATE SECTION SET titre = 'Choisir textile, stabilisateur et cadre', contenu = '{"intro":"La qualité d’une broderie dépend du trio textile, stabilisateur et cadre. Un tissu mal maintenu se déforme, fronce ou décale les couleurs.","objectives":["Associer stabilisateur et textile","Choisir le bon cadre","Tendre sans déformer"],"steps":[{"title":"Analyser le textile","text":"Évaluez extensibilité, épaisseur, surface et sens du tissu avant de choisir le renfort."},{"title":"Choisir le stabilisateur","text":"Découpable, déchirable ou hydrosoluble selon la matière et la densité du motif."},{"title":"Mettre en cadre","text":"Tendez uniformément sans étirer le textile et vérifiez que la zone utile est libre."}],"callouts":[{"type":"tip","title":"Textile extensible","text":"Un stabilisateur adapté empêche le motif de se resserrer ou de gondoler."},{"type":"warning","title":"Cadre mal serré","text":"Un déplacement du tissu peut casser l’aiguille ou décaler toute la broderie."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 1;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 1 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Brodeuse numérique · 1';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Brodeuse numérique.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Brodeuse numérique.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-1;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Choisir textile, stabilisateur et cadre', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Choisir textile, stabilisateur et cadre', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pourquoi utiliser un stabilisateur ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pourquoi utiliser un stabilisateur ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quels critères guident le choix du cadre ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quels critères guident le choix du cadre ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Limiter la déformation du textile pendant la broderie', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Augmenter la vitesse réseau', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Remplacer le fil supérieur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nettoyer l’écran', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Taille du motif', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Zone réellement brodable', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couleur du câble USB', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nombre de polices installées', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 2 : Préparer le motif, les couleurs et l’aiguille
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Préparer le motif, les couleurs et l’aiguille', '{"intro":"Le fichier doit respecter la zone du cadre, la densité de points et l’ordre des couleurs. L’aiguille et le fil doivent correspondre au textile.","objectives":["Vérifier le format de broderie","Contrôler densité et taille","Choisir fil et aiguille"],"steps":[{"title":"Inspecter le motif","text":"Vérifiez dimensions, nombre de points, ordre des couleurs et zones très denses."},{"title":"Choisir l’aiguille","text":"Adaptez type et diamètre au textile et au fil, puis remplacez toute aiguille tordue."},{"title":"Préparer les couleurs","text":"Identifiez chaque bobine et anticipez les changements pour éviter les erreurs d’ordre."}],"callouts":[{"type":"warning","title":"Redimensionnement","text":"Agrandir ou réduire fortement un motif sans recalcul des points peut dégrader la broderie."},{"type":"check","title":"Fichier prêt","text":"Motif dans le cadre, densité cohérente, ordre des couleurs relu et aiguille adaptée."}]}', NULL, 2, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 2);
UPDATE SECTION SET titre = 'Préparer le motif, les couleurs et l’aiguille', contenu = '{"intro":"Le fichier doit respecter la zone du cadre, la densité de points et l’ordre des couleurs. L’aiguille et le fil doivent correspondre au textile.","objectives":["Vérifier le format de broderie","Contrôler densité et taille","Choisir fil et aiguille"],"steps":[{"title":"Inspecter le motif","text":"Vérifiez dimensions, nombre de points, ordre des couleurs et zones très denses."},{"title":"Choisir l’aiguille","text":"Adaptez type et diamètre au textile et au fil, puis remplacez toute aiguille tordue."},{"title":"Préparer les couleurs","text":"Identifiez chaque bobine et anticipez les changements pour éviter les erreurs d’ordre."}],"callouts":[{"type":"warning","title":"Redimensionnement","text":"Agrandir ou réduire fortement un motif sans recalcul des points peut dégrader la broderie."},{"type":"check","title":"Fichier prêt","text":"Motif dans le cadre, densité cohérente, ordre des couleurs relu et aiguille adaptée."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 2;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 2 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Brodeuse numérique · 2';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Brodeuse numérique.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Brodeuse numérique.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-2;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Préparer le motif, les couleurs et l’aiguille', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Préparer le motif, les couleurs et l’aiguille', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Pourquoi un fort redimensionnement peut-il poser problème ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Pourquoi un fort redimensionnement peut-il poser problème ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Que faut-il contrôler dans le fichier avant lancement ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Que faut-il contrôler dans le fichier avant lancement ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La densité de points peut devenir inadaptée', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le cadre change de couleur', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Le fil devient conducteur', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'La machine perd son nom', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Dimensions et zone du cadre', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ordre des couleurs', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Luminosité de la pièce', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Mot de passe du compte', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 3 : Enfiler, cadrer et lancer un test
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Enfiler, cadrer et lancer un test', '{"intro":"Le chemin du fil, la canette et l’orientation du cadre doivent être corrects avant de démarrer. Un tracé de contour vérifie que l’aiguille restera dans la zone utile.","objectives":["Suivre le chemin d’enfilage","Installer la canette","Tracer la zone du motif"],"steps":[{"title":"Enfiler machine arrêtée","text":"Suivez tous les guides et tensions indiqués, puis passez correctement dans l’aiguille."},{"title":"Installer canette et cadre","text":"Vérifiez le sens de la canette et verrouillez le cadre sans tissu sous la zone mobile."},{"title":"Faire le contour","text":"Utilisez la fonction de traçage pour confirmer l’emplacement et l’absence d’obstacle."}],"callouts":[{"type":"warning","title":"Zone mobile","text":"Gardez mains, ciseaux et fils libres hors du cadre pendant les mouvements."},{"type":"tip","title":"Test sur chute","text":"Pour un nouveau textile, réalisez un petit essai avec le même stabilisateur."}]}', NULL, 3, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 3);
UPDATE SECTION SET titre = 'Enfiler, cadrer et lancer un test', contenu = '{"intro":"Le chemin du fil, la canette et l’orientation du cadre doivent être corrects avant de démarrer. Un tracé de contour vérifie que l’aiguille restera dans la zone utile.","objectives":["Suivre le chemin d’enfilage","Installer la canette","Tracer la zone du motif"],"steps":[{"title":"Enfiler machine arrêtée","text":"Suivez tous les guides et tensions indiqués, puis passez correctement dans l’aiguille."},{"title":"Installer canette et cadre","text":"Vérifiez le sens de la canette et verrouillez le cadre sans tissu sous la zone mobile."},{"title":"Faire le contour","text":"Utilisez la fonction de traçage pour confirmer l’emplacement et l’absence d’obstacle."}],"callouts":[{"type":"warning","title":"Zone mobile","text":"Gardez mains, ciseaux et fils libres hors du cadre pendant les mouvements."},{"type":"tip","title":"Test sur chute","text":"Pour un nouveau textile, réalisez un petit essai avec le même stabilisateur."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 3;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 3 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Brodeuse numérique · 3';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Brodeuse numérique.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Brodeuse numérique.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-3;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Enfiler, cadrer et lancer un test', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Enfiler, cadrer et lancer un test', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'À quoi sert le tracé de contour avant broderie ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'À quoi sert le tracé de contour avant broderie ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quand faut-il approcher les mains du cadre ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quand faut-il approcher les mains du cadre ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Vérifier la position et l’enveloppe du motif', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couper automatiquement les fils', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Changer la densité du fichier', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Remplir la canette', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Uniquement machine arrêtée et mouvement terminé', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pendant un changement rapide de direction', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Pendant la couture pour tendre le tissu', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Quand l’aiguille descend', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Section 4 : Surveiller, corriger et finir la broderie
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @parent_id, 'Surveiller, corriger et finir la broderie', '{"intro":"Pendant la broderie, surveillez tension, casse de fil, plis et bruit. Après la fin, retirez le cadre avant de nettoyer les fils et le stabilisateur.","objectives":["Réagir à une casse de fil","Éviter les nids de fil","Finir sans abîmer le textile"],"steps":[{"title":"Observer les premières zones","text":"Vérifiez que le tissu reste plat, que le fil couvre correctement et qu’aucune boucle ne se forme."},{"title":"Traiter une alerte","text":"Arrêtez selon la procédure, retirez le fil coincé sans forcer et réenfilez avant reprise."},{"title":"Finir proprement","text":"Retirez le cadre, coupez les fils à l’arrière et ôtez le stabilisateur selon son type."}],"callouts":[{"type":"warning","title":"Aiguille cassée","text":"Arrêtez immédiatement, retrouvez tous les fragments et signalez l’incident."},{"type":"check","title":"Poste rendu","text":"Canette et fils rangés, chutes retirées, cadre remis en place et anomalie signalée."}]}', NULL, 4, CURRENT_TIMESTAMP
WHERE @parent_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @parent_id AND ordre = 4);
UPDATE SECTION SET titre = 'Surveiller, corriger et finir la broderie', contenu = '{"intro":"Pendant la broderie, surveillez tension, casse de fil, plis et bruit. Après la fin, retirez le cadre avant de nettoyer les fils et le stabilisateur.","objectives":["Réagir à une casse de fil","Éviter les nids de fil","Finir sans abîmer le textile"],"steps":[{"title":"Observer les premières zones","text":"Vérifiez que le tissu reste plat, que le fil couvre correctement et qu’aucune boucle ne se forme."},{"title":"Traiter une alerte","text":"Arrêtez selon la procédure, retirez le fil coincé sans forcer et réenfilez avant reprise."},{"title":"Finir proprement","text":"Retirez le cadre, coupez les fils à l’arrière et ôtez le stabilisateur selon son type."}],"callouts":[{"type":"warning","title":"Aiguille cassée","text":"Arrêtez immédiatement, retrouvez tous les fragments et signalez l’incident."},{"type":"check","title":"Poste rendu","text":"Canette et fils rangés, chutes retirées, cadre remis en place et anomalie signalée."}]}', videoUrl = NULL WHERE formationId = @parent_id AND ordre = 4;
SET @parent_section_id = (SELECT id FROM SECTION WHERE formationId = @parent_id AND ordre = 4 ORDER BY id LIMIT 1);
SET @internal_title = '[FABOS SECTION] Brodeuse numérique · 4';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @internal_title, 'Mini-quiz de validation de section pour Brodeuse numérique.', parent.image, 'Quiz interne', parent.niveau, '3 min', 'FabOS', NULL, 'Valider les acquis de la section.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;'), 'Validation intégrée au parcours'
FROM FORMATION parent WHERE parent.id = @parent_id AND @parent_section_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne');
SET @internal_formation_id = (SELECT id FROM FORMATION WHERE titre = @internal_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Mini-quiz de validation de section pour Brodeuse numérique.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_SECTION_ID=', @parent_section_id, ';FABOS_QUIZ_CONTEXT=section;FABOS_BONUS=0;FABOS_QUIZ_KEY=section-4;') WHERE id = @internal_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @internal_formation_id, 'Validation · Surveiller, corriger et finir la broderie', 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.', NULL, 1, CURRENT_TIMESTAMP
WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Validation · Surveiller, corriger et finir la broderie', contenu = 'Mini-quiz distinct des quiz de la page. Sa réussite déverrouille la section suivante.' WHERE formationId = @internal_formation_id AND ordre = 1;
SET @internal_section_id = (SELECT id FROM SECTION WHERE formationId = @internal_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @internal_formation_id, @internal_section_id, 80 WHERE @internal_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @internal_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @internal_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @internal_section_id, noteMinimale = 80 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Que faire en cas de casse d’aiguille ?', 'single', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Que faire en cas de casse d’aiguille ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Quelles actions terminent correctement la broderie ?', 'multiple', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Quelles actions terminent correctement la broderie ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 2;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Arrêter, récupérer les fragments et signaler', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Continuer avec un morceau manquant', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Mettre les mains sous le cadre en mouvement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Augmenter la vitesse', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Retirer le cadre avant les finitions', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Éliminer le stabilisateur selon son type', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Tirer fortement sur les fils pendant le mouvement', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Laisser les chutes dans la machine', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;

-- Quiz bonus : Défi expert · diagnostiquer une broderie imparfaite
SET @bonus_title = '[FABOS BONUS] Brodeuse numérique · Défi expert';
INSERT INTO FORMATION (badgeId, titre, description, image, categorie, niveau, duree, formateur, placesTotales, objectifs, prerequis, materielFourni)
SELECT parent.badgeId, @bonus_title, 'Quiz bonus facultatif pour Brodeuse numérique. Il ne compte pas dans la progression.', parent.image, 'Quiz interne', parent.niveau, '4 min', 'FabOS', NULL, 'Approfondir le diagnostic machine.', CONCAT('FABOS_PARENT_FORMATION_ID=', parent.id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;'), 'Quiz bonus'
FROM FORMATION parent WHERE parent.id = @parent_id AND NOT EXISTS (SELECT 1 FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne');
SET @bonus_formation_id = (SELECT id FROM FORMATION WHERE titre = @bonus_title AND categorie = 'Quiz interne' ORDER BY id LIMIT 1);
UPDATE FORMATION SET description = 'Quiz bonus facultatif pour Brodeuse numérique. Il ne compte pas dans la progression.', prerequis = CONCAT('FABOS_PARENT_FORMATION_ID=', @parent_id, ';FABOS_QUIZ_CONTEXT=page;FABOS_BONUS=1;FABOS_QUIZ_KEY=bonus-expert;') WHERE id = @bonus_formation_id;
INSERT INTO SECTION (formationId, titre, contenu, videoUrl, ordre, createdAt)
SELECT @bonus_formation_id, 'Défi expert · diagnostiquer une broderie imparfaite', 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.', NULL, 1, CURRENT_TIMESTAMP
WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1);
UPDATE SECTION SET titre = 'Défi expert · diagnostiquer une broderie imparfaite', contenu = 'Défi facultatif : le score est enregistré mais exclu de la progression obligatoire.' WHERE formationId = @bonus_formation_id AND ordre = 1;
SET @bonus_section_id = (SELECT id FROM SECTION WHERE formationId = @bonus_formation_id AND ordre = 1 ORDER BY id LIMIT 1);
INSERT INTO QUIZ (formationId, sectionId, noteMinimale) SELECT @bonus_formation_id, @bonus_section_id, 67 WHERE @bonus_formation_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUIZ WHERE formationId = @bonus_formation_id);
SET @quiz_id = (SELECT id FROM QUIZ WHERE formationId = @bonus_formation_id ORDER BY id LIMIT 1);
UPDATE QUIZ SET sectionId = @bonus_section_id, noteMinimale = 67 WHERE id = @quiz_id;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Le tissu fronce autour du motif. Quelles causes sont plausibles ?', 'multiple', 1 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 1);
UPDATE QUESTION SET texte = 'Le tissu fronce autour du motif. Quelles causes sont plausibles ?', type = 'multiple' WHERE quizId = @quiz_id AND ordre = 1;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Des boucles apparaissent sous le tissu. Quelle piste vérifier d’abord ?', 'single', 2 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 2);
UPDATE QUESTION SET texte = 'Des boucles apparaissent sous le tissu. Quelle piste vérifier d’abord ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 2;
INSERT INTO QUESTION (quizId, texte, type, ordre) SELECT @quiz_id, 'Les couleurs sont décalées entre deux zones. Quelle cause est directe ?', 'single', 3 WHERE @quiz_id IS NOT NULL AND NOT EXISTS (SELECT 1 FROM QUESTION WHERE quizId = @quiz_id AND ordre = 3);
UPDATE QUESTION SET texte = 'Les couleurs sont décalées entre deux zones. Quelle cause est directe ?', type = 'single' WHERE quizId = @quiz_id AND ordre = 3;
DELETE c FROM CHOIX c INNER JOIN QUESTION q ON q.id = c.questionId WHERE q.quizId = @quiz_id;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Stabilisation insuffisante', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Densité trop forte ou tension inadaptée', 1, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nom du fichier trop court', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Cadre trop grand à l’écran', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 1 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Enfilage supérieur et passage dans les tensions', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Couleur du fil de canette', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Nombre d’utilisateurs', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Ordre des menus', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 2 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Tissu ou cadre ayant bougé', 1, 1 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Motif trop récent', 0, 2 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Canette pleine', 0, 3 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;
INSERT INTO CHOIX (questionId, texte, estCorrect, ordre) SELECT q.id, 'Écran en mode sombre', 0, 4 FROM QUESTION q WHERE q.quizId = @quiz_id AND q.ordre = 3 LIMIT 1;

COMMIT;
