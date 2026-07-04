-- FABOS - enrichissement simple FORMATION
-- SQL controle et idempotent: ajoute uniquement des colonnes nullable puis remplit la formation 3D existante.

ALTER TABLE FORMATION
    ADD COLUMN IF NOT EXISTS categorie VARCHAR(100) NULL AFTER image,
    ADD COLUMN IF NOT EXISTS niveau INT NULL AFTER categorie,
    ADD COLUMN IF NOT EXISTS duree VARCHAR(100) NULL AFTER niveau,
    ADD COLUMN IF NOT EXISTS formateur VARCHAR(150) NULL AFTER duree,
    ADD COLUMN IF NOT EXISTS placesTotales INT NULL AFTER formateur,
    ADD COLUMN IF NOT EXISTS objectifs TEXT NULL AFTER placesTotales,
    ADD COLUMN IF NOT EXISTS prerequis TEXT NULL AFTER objectifs,
    ADD COLUMN IF NOT EXISTS materielFourni TEXT NULL AFTER prerequis;

UPDATE FORMATION
SET categorie = 'Impression 3D',
    niveau = 1,
    duree = '2h',
    formateur = 'Équipe FabLab',
    placesTotales = 8,
    objectifs = 'Comprendre les règles de sécurité liées à l’impression 3D. Préparer un modèle dans le slicer. Choisir les paramètres de base pour le PLA. Lancer, surveiller et retirer une impression simple.',
    prerequis = 'Aucun prérequis technique obligatoire. Bases de sécurité atelier et compte FabOS actif recommandés.',
    materielFourni = 'Imprimante 3D, filament PLA, outils de calibration, fiche sécurité.'
WHERE titre = 'Formation imprimante 3D';
