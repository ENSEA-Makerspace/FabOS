-- FABOS — validations physiques liées aux formations existantes
-- Script idempotent : aucune table ni colonne n'est créée ou modifiée.
-- Il ajoute uniquement une FORMATION interne de validation physique par formation visible dotée d'un badge.

SET NAMES utf8mb4;

INSERT INTO FORMATION (
    badgeId,
    titre,
    description,
    image,
    categorie,
    niveau,
    duree,
    formateur,
    placesTotales,
    objectifs,
    prerequis,
    materielFourni
)
SELECT
    NULL,
    CONCAT('Validation physique — ', parent.titre),
    CONCAT('Validation en présentiel obligatoire avant l’attribution définitive du badge « ', badge.nom, ' ». Un membre du FabLab confirme la prise en main, la sécurité et les gestes essentiels.'),
    parent.image,
    'Validation physique',
    parent.niveau,
    '30 min',
    'Équipe FabLab',
    NULL,
    'Valider en présentiel la prise en main, les règles de sécurité et la réaction en cas d’incident.',
    CONCAT('FABOS_PHYSICAL_PARENT_FORMATION_ID=', parent.id, ';'),
    'Machine, équipements de protection et fiche de validation.'
FROM FORMATION parent
INNER JOIN BADGE badge ON badge.id = parent.badgeId
WHERE (parent.categorie IS NULL OR parent.categorie NOT IN ('Quiz interne', 'Validation physique'))
  AND NOT EXISTS (
      SELECT 1
      FROM FORMATION physical
      WHERE physical.categorie = 'Validation physique'
        AND physical.prerequis LIKE CONCAT('%FABOS_PHYSICAL_PARENT_FORMATION_ID=', parent.id, ';%')
  );

SELECT
    parent.id AS formation_id,
    parent.titre AS formation,
    physical.id AS validation_physique_id,
    physical.titre AS validation_physique
FROM FORMATION parent
LEFT JOIN FORMATION physical
    ON physical.categorie = 'Validation physique'
   AND physical.prerequis LIKE CONCAT('%FABOS_PHYSICAL_PARENT_FORMATION_ID=', parent.id, ';%')
WHERE parent.badgeId IS NOT NULL
  AND (parent.categorie IS NULL OR parent.categorie NOT IN ('Quiz interne', 'Validation physique'))
ORDER BY parent.id;
