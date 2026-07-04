-- FABOS - enrichissement visuel MACHINE
-- SQL controle et idempotent: ajoute uniquement des colonnes nullable puis remplit par machineToken.

ALTER TABLE MACHINE
    ADD COLUMN IF NOT EXISTS categorySlug VARCHAR(100) NULL AFTER granularite,
    ADD COLUMN IF NOT EXISTS categoryLabel VARCHAR(100) NULL AFTER categorySlug,
    ADD COLUMN IF NOT EXISTS levelSlug VARCHAR(50) NULL AFTER categoryLabel,
    ADD COLUMN IF NOT EXISTS levelLabel VARCHAR(50) NULL AFTER levelSlug,
    ADD COLUMN IF NOT EXISTS iconSlug VARCHAR(50) NULL AFTER levelLabel,
    ADD COLUMN IF NOT EXISTS materials JSON NULL AFTER iconSlug,
    ADD COLUMN IF NOT EXISTS features JSON NULL AFTER materials,
    ADD COLUMN IF NOT EXISTS requirementTitle VARCHAR(255) NULL AFTER features,
    ADD COLUMN IF NOT EXISTS requirementDescription TEXT NULL AFTER requirementTitle,
    ADD COLUMN IF NOT EXISTS popularity INT NULL AFTER requirementDescription;

UPDATE MACHINE
SET categorySlug = 'impression-3d',
    categoryLabel = 'Impression 3D',
    levelSlug = 'niveau-1',
    levelLabel = 'Niveau 1',
    iconSlug = 'impression-3d',
    materials = JSON_ARRAY('PLA', 'PETG', 'TPU', 'Support'),
    features = JSON_ARRAY('Volume utile standard FabLab', 'Réservation par créneau', 'Utilisation accompagnée possible', 'Traçabilité RFID'),
    requirementTitle = 'Formation imprimante 3D recommandée',
    requirementDescription = 'Validation formation ou accompagnement staff conseillé avant première utilisation.',
    popularity = 4
WHERE machineToken IN ('printer-01', 'ultimaker-s5-01', 'prusa-mk3s-01');

UPDATE MACHINE
SET categorySlug = 'decoupe',
    categoryLabel = 'Découpe',
    levelSlug = 'niveau-2',
    levelLabel = 'Niveau 2',
    iconSlug = 'decoupe',
    materials = JSON_ARRAY('Bois fin', 'Acrylique', 'Carton', 'Vinyle'),
    features = JSON_ARRAY('Plan de coupe vectoriel', 'Extraction active', 'Paramètres matière', 'Surveillance obligatoire'),
    requirementTitle = 'Formation découpe recommandée',
    requirementDescription = 'Contrôle des matériaux et consignes sécurité avant réservation autonome.',
    popularity = 4
WHERE machineToken IN ('laser-co2-01', 'vinyl-cutter-01');

UPDATE MACHINE
SET categorySlug = 'electronique',
    categoryLabel = 'Électronique',
    levelSlug = 'niveau-1',
    levelLabel = 'Niveau 1',
    iconSlug = 'electronique',
    materials = JSON_ARRAY('PCB', 'Composants traversants', 'CMS', 'Signaux basse tension'),
    features = JSON_ARRAY('Poste outillé', 'Mesure et diagnostic', 'Consommables disponibles', 'Zone ventilée'),
    requirementTitle = 'Brief sécurité électronique',
    requirementDescription = 'Respect des tensions autorisées et rangement du poste après usage.',
    popularity = 4
WHERE machineToken IN ('station-soudure-01', 'oscilloscope-01');

UPDATE MACHINE
SET categorySlug = 'usinage',
    categoryLabel = 'Usinage',
    levelSlug = 'niveau-3',
    levelLabel = 'Niveau 3',
    iconSlug = 'usinage',
    materials = JSON_ARRAY('Bois', 'Plastiques techniques', 'Aluminium léger', 'Mousses'),
    features = JSON_ARRAY('Parcours outil CAM', 'Bridage obligatoire', 'Aspiration copeaux', 'Supervision staff'),
    requirementTitle = 'Autorisation staff requise',
    requirementDescription = 'Machine à risque : réservation à confirmer avec un encadrant.',
    popularity = 3
WHERE machineToken = 'cnc-fraiseuse-01';

UPDATE MACHINE
SET categorySlug = 'textile',
    categoryLabel = 'Textile',
    levelSlug = 'niveau-2',
    levelLabel = 'Niveau 2',
    iconSlug = 'textile',
    materials = JSON_ARRAY('Coton', 'Feutrine', 'Écussons', 'Fils polyester'),
    features = JSON_ARRAY('Fichiers de broderie', 'Cadres multiples', 'Changement de fil', 'Tests sur chute'),
    requirementTitle = 'Initiation textile recommandee',
    requirementDescription = 'Préparation du fichier et choix du support à vérifier avant production.',
    popularity = 4
WHERE machineToken = 'brodeuse-01';
