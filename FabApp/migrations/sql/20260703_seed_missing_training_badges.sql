-- FABOS - badges manquants pour formations non-3D et conversion MACHINE_BADGE
-- Script idempotent : ne duplique ni BADGE, ni MACHINE_BADGE, ni UTILISATEUR_BADGE.

INSERT INTO BADGE (nom, description, icone)
SELECT 'Badge Laser CO2', 'Badge obtenu après validation de la formation découpe laser CO2.', 'laser'
WHERE NOT EXISTS (SELECT 1 FROM BADGE WHERE nom = 'Badge Laser CO2');

INSERT INTO BADGE (nom, description, icone)
SELECT 'Badge Soudure Électronique', 'Badge obtenu après validation de la formation soudure électronique.', 'soldering'
WHERE NOT EXISTS (SELECT 1 FROM BADGE WHERE nom = 'Badge Soudure Électronique');

INSERT INTO BADGE (nom, description, icone)
SELECT 'Badge Découpe Vinyle', 'Badge obtenu après validation de la formation découpe vinyle.', 'vinyl'
WHERE NOT EXISTS (SELECT 1 FROM BADGE WHERE nom = 'Badge Découpe Vinyle');

INSERT INTO BADGE (nom, description, icone)
SELECT 'Badge CNC', 'Badge obtenu après validation de la formation fraiseuse CNC.', 'cnc'
WHERE NOT EXISTS (SELECT 1 FROM BADGE WHERE nom = 'Badge CNC');

INSERT INTO BADGE (nom, description, icone)
SELECT 'Badge Oscilloscope', 'Badge obtenu après validation de la formation oscilloscope numérique.', 'oscilloscope'
WHERE NOT EXISTS (SELECT 1 FROM BADGE WHERE nom = 'Badge Oscilloscope');

INSERT INTO BADGE (nom, description, icone)
SELECT 'Badge Brodeuse Numérique', 'Badge obtenu après validation de la formation brodeuse numérique.', 'embroidery'
WHERE NOT EXISTS (SELECT 1 FROM BADGE WHERE nom = 'Badge Brodeuse Numérique');

UPDATE FORMATION f
JOIN BADGE b ON b.nom = 'Badge Laser CO2'
SET f.badgeId = b.id
WHERE f.titre = 'Formation découpe laser CO2'
  AND (f.badgeId IS NULL OR f.badgeId <> b.id);

UPDATE FORMATION f
JOIN BADGE b ON b.nom = 'Badge Soudure Électronique'
SET f.badgeId = b.id
WHERE f.titre = 'Formation soudure électronique'
  AND (f.badgeId IS NULL OR f.badgeId <> b.id);

UPDATE FORMATION f
JOIN BADGE b ON b.nom = 'Badge Découpe Vinyle'
SET f.badgeId = b.id
WHERE f.titre = 'Formation découpe vinyle'
  AND (f.badgeId IS NULL OR f.badgeId <> b.id);

UPDATE FORMATION f
JOIN BADGE b ON b.nom = 'Badge CNC'
SET f.badgeId = b.id
WHERE f.titre = 'Formation fraiseuse CNC'
  AND (f.badgeId IS NULL OR f.badgeId <> b.id);

UPDATE FORMATION f
JOIN BADGE b ON b.nom = 'Badge Oscilloscope'
SET f.badgeId = b.id
WHERE f.titre = 'Formation oscilloscope numérique'
  AND (f.badgeId IS NULL OR f.badgeId <> b.id);

UPDATE FORMATION f
JOIN BADGE b ON b.nom = 'Badge Brodeuse Numérique'
SET f.badgeId = b.id
WHERE f.titre = 'Formation brodeuse numérique'
  AND (f.badgeId IS NULL OR f.badgeId <> b.id);

INSERT INTO MACHINE_BADGE (machineId, badgeId, requiredForAccess)
SELECT DISTINCT mf.machineId, f.badgeId, 1
FROM MACHINE_FORMATION mf
JOIN FORMATION f ON f.id = mf.formationId
WHERE mf.requiredForAccess = 1
  AND f.badgeId IS NOT NULL
  AND NOT EXISTS (
      SELECT 1 FROM MACHINE_BADGE mb
      WHERE mb.machineId = mf.machineId AND mb.badgeId = f.badgeId
  );

INSERT INTO UTILISATEUR_BADGE (utilisateurId, badgeId)
SELECT DISTINCT p.userId, f.badgeId
FROM PROGRESSION p
JOIN FORMATION f ON f.id = p.formationId
WHERE p.completed = 1
  AND f.badgeId IS NOT NULL
  AND NOT EXISTS (
      SELECT 1 FROM UTILISATEUR_BADGE ub
      WHERE ub.utilisateurId = p.userId AND ub.badgeId = f.badgeId
  );
